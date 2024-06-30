#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>

class DifferentialDrive {
public:
    DifferentialDrive(ros::NodeHandle& nh) 
        : wheel_radius(0.035), wheel_base_distance(0.23),
          Xk(Eigen::Vector3d::Zero()), 
          lin_vel(0.0), ang_vel(0.0), dt(0.0), goal(0),
          left_wheel_velocity(0.0), right_wheel_velocity(0.0), left_wheel_received(false), gb(0)
    {
        Pk = Eigen::Matrix3d::Identity() * 0.04;
        Qk = (Eigen::Matrix2d() << 0.2*0.2, 0.0, 0.0, 0.2*0.2).finished();
        Rk = (Eigen::Matrix2d() << 2.0, 0.0, 0.0, 2.0).finished();

        js_sub = nh.subscribe("/velocities", 10, &DifferentialDrive::jointStateCallback, this);
        goal_sub = nh.subscribe("/goal_set", 10, &DifferentialDrive::goalSetCallback, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("kobuki/odom", 10);
    }

    void goalSetCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        goal = 1;
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (goal == 1) {
            if (gb == 0) {
                last_time = ros::Time::now();
                gb = 1;
            }

            right_wheel_velocity = msg->velocity[0];
            left_wheel_velocity = msg->velocity[1];

            double left_lin_vel = left_wheel_velocity * wheel_radius;
            double right_lin_vel = right_wheel_velocity * wheel_radius;

            v = (left_lin_vel + right_lin_vel) / 2.0;
            w = (right_lin_vel - left_lin_vel) / wheel_base_distance;

            current_time = msg->header.stamp;
            dt = (current_time - last_time).toSec();
            last_time = current_time;

            Xk[0] += std::cos(Xk[2]) * v * dt;
            Xk[1] += std::sin(Xk[2]) * v * dt;
            Xk[2] += w * dt;

            Eigen::Matrix3d Ak;
            Ak << 1, 0, -std::sin(Xk[2]) * v * dt,
                  0, 1, std::cos(Xk[2]) * v * dt,
                  0, 0, 1;

            Eigen::Matrix<double, 3, 2> Bk;
            Bk << std::cos(Xk[2]) * dt * 0.5, std::cos(Xk[2]) * dt * 0.5,
                  std::sin(Xk[2]) * dt * 0.5, std::sin(Xk[2]) * dt * 0.5,
                  (dt * wheel_radius) / wheel_base_distance, -(dt * wheel_radius) / wheel_base_distance;

            Pk = Ak * Pk * Ak.transpose() + Bk * Qk * Bk.transpose();

            publishOdometry();
        }
    }

    void publishOdometry() {
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "world_ned";
        odom.child_frame_id = "turtlebot/kobuki/base_footprint";

        odom.pose.pose.position.x = Xk[0];
        odom.pose.pose.position.y = Xk[1];

        tf::Quaternion q = tf::createQuaternionFromYaw(Xk[2]);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = w;

        odom.pose.covariance = {Pk(0, 0), Pk(0, 1), 0.0, 0.0, 0.0, Pk(0, 2),
                                Pk(1, 0), Pk(1, 1), 0.0, 0.0, 0.0, Pk(1, 2),
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                Pk(2, 0), Pk(2, 1), 0.0, 0.0, 0.0, Pk(2, 2)};

        odom_pub.publish(odom);

        tf_br.sendTransform(tf::StampedTransform(tf::Transform(q, tf::Vector3(Xk[0], Xk[1], 0.0)),
                                                 ros::Time::now(), odom.header.frame_id, odom.child_frame_id));
    }

private:
    double wheel_radius;
    double wheel_base_distance;
    Eigen::Vector3d Xk;
    double lin_vel;
    double ang_vel;
    double dt;
    int goal;

    double left_wheel_velocity;
    double right_wheel_velocity;
    bool left_wheel_received;

    int gb;
    Eigen::Matrix3d Pk;
    Eigen::Matrix2d Qk;
    Eigen::Matrix2d Rk;

    ros::Time last_time;
    ros::Time current_time;

    double v;
    double w;

    ros::Subscriber js_sub;
    ros::Subscriber goal_sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster tf_br;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "differential_drive");
    ros::NodeHandle nh;
    DifferentialDrive robot(nh);
    ros::spin();
    return 0;
}
