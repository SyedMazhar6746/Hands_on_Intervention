#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transformations.h>

// Importing function definitions
#include "func_define.hpp"
#include "func_call_2.hpp"

class RobotModel {
private:
    double wheel_base_distance = 0.23;
    double wheel_radius = 0.035;
    std::array<double, 3> state {0.0, 0.0, 0.0};
    double dt = 0.0;
    int t = 0;

    // Task definition
    double q_max = 0.5;
    double q_max_a = q_max + 0.01;
    double q_min = -0.5;
    double q_min_a = q_min - 0.01;
    double zero_vel = 0.0;

    std::vector<Task*> tasks;
    std::vector<double> weight;
    std::vector<double> goal;

    // ROS Publishers
    ros::Publisher pose_EE_pub;
    ros::Publisher goal_check;
    ros::Publisher joint_velocity;
    ros::Publisher wheel_velocity;
    ros::Publisher J_wheel_velocity;

    // ROS Subscribers
    ros::Subscriber weight_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber task_sub;
    ros::Subscriber joints_sub;

public:
    RobotModel(ros::NodeHandle& nh)
        : pose_EE_pub(nh.advertise<geometry_msgs::PoseStamped>("pose_EE", 10)),
          goal_check(nh.advertise<geometry_msgs::PoseStamped>("/goal_check", 10)),
          joint_velocity(nh.advertise<std_msgs::Float64MultiArray>("/turtlebot/swiftpro/joint_velocity_controller/command", 10)),
          wheel_velocity(nh.advertise<std_msgs::Float64MultiArray>("/turtlebot/kobuki/commands/wheel_velocities", 10)),
          J_wheel_velocity(nh.advertise<sensor_msgs::JointState>("/velocities", 10)),
          weight_sub(nh.subscribe("/weight_set", 10, &RobotModel::weight_set_callback, this)),
          goal_sub(nh.subscribe("/goal_set", 10, &RobotModel::goals_set_callback, this)),
          task_sub(nh.subscribe("/task_set", 10, &RobotModel::task_set_callback, this)),
          joints_sub(nh.subscribe("/turtlebot/joint_states", 10, &RobotModel::joint_states_callback, this)) {
        ROS_INFO("RobotModel node initialized.");
    }

    void task_set_callback(const std_msgs::Int32::ConstPtr& task_index) {
        t = task_index->data;
        goals_set();
    }

    void goals_set_callback(const std_msgs::Float64MultiArray::ConstPtr& goal_msg) {
        goal = {goal_msg->data[0], goal_msg->data[1], goal_msg->data[2], goal_msg->data[3]};
        geometry_msgs::PoseStamped g_pose = goal_pose({goal[0], goal[1], goal[2]}, {goal[3]});
        goal_check.publish(g_pose);

        if (t == 0) {
            ROS_INFO("Goal and task set.");
            tasks = {
                new Position3D("End-effector position", {goal[0], goal[1], goal[2]}, 6)
            };
        }
    }

    void weight_set_callback(const std_msgs::Float64MultiArray::ConstPtr& weight_msg) {
        weight = weight_msg->data;
    }

    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& data) {
        if (!goal.empty()) {
            std::vector<std::string> names {"turtlebot/swiftpro/joint1", "turtlebot/swiftpro/joint2", "turtlebot/swiftpro/joint3", "turtlebot/swiftpro/joint4"};
            if (data->name == names) {
                Eigen::VectorXd theta(data->position.size());
                for (size_t i = 0; i < data->position.size(); ++i) {
                    theta(i) = data->position[i];
                }

                Manipulator robot(theta);
                dt = state[0];
                Eigen::MatrixXd dq(robot.getDOF(), 1);
                dq.setZero();

                robot.update(dq, dt, {state[0], state[1], state[2]});

                Eigen::MatrixXd P = Eigen::MatrixXd::Identity(robot.getDOF(), robot.getDOF());
                for (auto& task : tasks) {
                    task->update(&robot);
                    if (task->bool_is_Active()) {
                        Eigen::VectorXd err = task->getError();
                        Eigen::MatrixXd J = task->getJacobian();
                        Eigen::MatrixXd J_bar = J * P;

                        Eigen::MatrixXd J_DLS = W_DLS(J_bar, 0.1, weight);
                        Eigen::MatrixXd J_pinv = J_bar.completeOrthogonalDecomposition().pseudoInverse();

                        dq = dq + J_DLS * (err - J * dq);

                        P = P - J_pinv * J_bar;

                        double p_v = 0.3;
                        double n_v = -0.3;
                        for (int m = 0; m < dq.rows(); ++m) {
                            if (dq(m) < n_v) {
                                dq(m) = scale(dq, n_v, m);
                            }
                            if (dq(m) > p_v) {
                                dq(m) = scale(dq, p_v, m);
                            }
                        }
                    }
                }

                send_velocity(dq);

                geometry_msgs::PoseStamped pose_eef = pose_EE(robot.getEETransform());
                pose_EE_pub.publish(pose_eef);

                robot.update(dq, dt, {state[0], state[1], state[2]});
            }
        }
    }

    void send_velocity(const Eigen::MatrixXd& q) {
        std_msgs::Float64MultiArray p;
        p.data.resize(4);
        for (int i = 0; i < 4; ++i) {
            p.data[i] = q(i + 2, 0);
        }
        joint_velocity.publish(p);

        double w = q(0, 0);
        double v = q(1, 0);
        double v_r = (2 * v + w * wheel_base_distance) / (2 * wheel_radius);
        double v_l = (2 * v - w * wheel_base_distance) / (2 * wheel_radius);

        std_msgs::Float64MultiArray m;
        m.data = {v_r, v_l};
        wheel_velocity.publish(m);

        sensor_msgs::JointState j_v;
        j_v.header.stamp = ros::Time::now();
        j_v.velocity = {v_r, v_l};
        J_wheel_velocity.publish(j_v);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "kinematics_node");
    ros::NodeHandle nh;

    RobotModel robot(nh);

    ros::spin();

    return 0;
}
