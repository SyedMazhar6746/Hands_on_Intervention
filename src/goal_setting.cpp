#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

void goal_set(ros::Publisher& pub_goal, ros::Publisher& pub_weight) {
    std_msgs::Float64MultiArray move;
    std_msgs::Float64MultiArray weighted_DLS;
    
    ros::Rate rate(10);  // 10 Hz
    while (ros::ok()) {
        std::vector<double> goal;
        ros::param::get("goal", goal);
        if (goal.size() == 4) {
            move.data = goal;
            pub_goal.publish(move);
        } else {
            ROS_WARN("Goal parameter has incorrect size.");
        }

        std::vector<double> weights;
        ros::param::get("weighted_DLS", weights);
        if (weights.size() == 6) {
            weighted_DLS.data = weights;
            pub_weight.publish(weighted_DLS);
        } else {
            ROS_WARN("Weighted DLS parameter has incorrect size.");
        }

        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_node");
    ros::NodeHandle nh;

    nh.setParam("goal", std::vector<double>{0.80, -0.01, -0.25, 0.5});
    nh.setParam("weighted_DLS", std::vector<double>{1000, 1000, 1, 1, 1, 1});

    ros::Publisher pub_goal = nh.advertise<std_msgs::Float64MultiArray>("/goal_set", 10);
    ros::Publisher pub_weight = nh.advertise<std_msgs::Float64MultiArray>("/weight_set", 10);

    goal_set(pub_goal, pub_weight);

    ros::spin();
    return 0;
}
