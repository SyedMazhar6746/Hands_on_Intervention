#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <h_invention/intervention.h>  // Ensure the correct path to the header
#include <iostream>

class ManageObject {
public:
    ManageObject() {
        // Initialize publishers
        pub_weight = nh.advertise<std_msgs::Float64MultiArray>("/weight_set", 10);
        pub_goal = nh.advertise<std_msgs::Float64MultiArray>("/goal_set", 10);
        pub_aruco = nh.advertise<geometry_msgs::Point>("/get_aruco_pose", 10);
        pub_task = nh.advertise<std_msgs::Int32>("/task_set", 10);

        // Initialize services
        server_weight = nh.advertiseService("weight_server", &ManageObject::weight_set, this);
        server_aruco = nh.advertiseService("aruco_server", &ManageObject::aruco_set, this);
        server_goal = nh.advertiseService("goal_server", &ManageObject::goal_set, this);
        server_task = nh.advertiseService("task_server", &ManageObject::task_set, this);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub_weight;
    ros::Publisher pub_goal;
    ros::Publisher pub_aruco;
    ros::Publisher pub_task;

    ros::ServiceServer server_weight;
    ros::ServiceServer server_aruco;
    ros::ServiceServer server_goal;
    ros::ServiceServer server_task;

    bool weight_set(h_invention::intervention::Request &req, h_invention::intervention::Response &res) {
        auto weight_gain = req.data;
        std::cout << "weight: " << weight_gain << std::endl;

        // Setting the parameter
        nh.setParam("weighted_DLS", weight_gain);

        // Publishing the set value
        std_msgs::Float64MultiArray weighted_DLS;
        weighted_DLS.data = weight_gain;
        pub_weight.publish(weighted_DLS);

        return true;
    }

    bool task_set(h_invention::intervention::Request &req, h_invention::intervention::Response &res) {
        int selected_index = static_cast<int>(req.data[0]);
        std::cout << "selected_index: " << selected_index << std::endl;

        // Publishing the set value
        std_msgs::Int32 task_va;
        task_va.data = selected_index;
        pub_task.publish(task_va);

        return true;
    }

    bool goal_set(h_invention::intervention::Request &req, h_invention::intervention::Response &res) {
        auto goal = req.data;
        std::cout << "goal: " << goal << std::endl;

        // Setting the parameter
        nh.setParam("goal", goal);

        // Publishing the set value
        std_msgs::Float64MultiArray goal_value;
        goal_value.data = goal;
        pub_goal.publish(goal_value);

        return true;
    }

    bool aruco_set(h_invention::intervention::Request &req, h_invention::intervention::Response &res) {
        geometry_msgs::Point point_msg;
        point_msg.x = 1.0;
        pub_aruco.publish(point_msg);

        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "intervention_service");
    ManageObject manage_object;
    ros::spin();
    return 0;
}
