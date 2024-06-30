#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

class DifferentialDrive {
public:
    DifferentialDrive() : wheel_radius(0.035), wheel_base_distance(0.23), lin_vel(0.0), ang_vel(0.0), left_wheel_velocity(0.0), right_wheel_velocity(0.0), left_wheel_received(false) {
        Xk = {0.0, 0.0, 0.0};
        
        Pk = {0.04, 0, 0,
              0, 0.04, 0,
              0, 0, 0.04};
        
        Qk = {0.2*0.2, 0,
              0, 0.2*0.2};
        
        Rk = {2.0, 0,
              0, 2.0};

        js_sub = nh.subscribe("/turtlebot/stonefish_simulator/ground_truth_odometry", 10, &DifferentialDrive::jointStateCallback, this);
        gro_tru_pub = nh.advertise<std_msgs::Float64MultiArray>("/my_ground_truth", 10);
    }

    void jointStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        tf::Quaternion quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        double roll, pitch, yaw;
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        ROS_INFO("odom: x = %f, y = %f, yaw = %f", x, y, yaw);

        std_msgs::Float64MultiArray my_gro_tru;
        my_gro_tru.data.push_back(x);
        my_gro_tru.data.push_back(y);
        my_gro_tru.data.push_back(yaw);
        gro_tru_pub.publish(my_gro_tru);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(quaternion);
        tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world_ned", "turtlebot/kobuki/base_footprint"));
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber js_sub;
    ros::Publisher gro_tru_pub;
    tf::TransformBroadcaster tf_br;

    double wheel_radius;
    double wheel_base_distance;
    double lin_vel;
    double ang_vel;
    double left_wheel_velocity;
    double right_wheel_velocity;
    bool left_wheel_received;

    std::vector<double> Xk;
    std::vector<double> Pk;
    std::vector<double> Qk;
    std::vector<double> Rk;

    ros::Time last_time;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "differential_drive2");
    DifferentialDrive robot;
    ros::spin();
    return 0;
}
