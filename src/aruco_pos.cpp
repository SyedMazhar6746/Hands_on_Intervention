#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

class CameraImage {
public:
    CameraImage() {
        image_pub = nh.advertise<std_msgs::Float64MultiArray>("/small_aruco_position", 10);
        image_sub = nh.subscribe("/get_aruco_pose", 10, &CameraImage::uselessCallback, this);
        camera_param_sub = nh.subscribe("/turtlebot/kobuki/sensors/realsense/color/image_color", 10, &CameraImage::imageCallback, this);
        rate = ros::Rate(10);

        // Camera position with respect to the robot
        camera = {0.122, -0.033, 0.082, M_PI/2, 0.0, M_PI/2};
        marker_detected = false;
    }

    void uselessCallback(const geometry_msgs::Point::ConstPtr& useless) {
        // For simulation use the below line
        image_sub = nh.subscribe("/turtlebot/kobuki/sensors/realsense/color/image_color", 10, &CameraImage::imageCallback, this);
        // For reality use the below line
        // image_sub = nh.subscribe("/turtlebot/kobuki/realsense/color/image_raw", 10, &CameraImage::imageCallback, this);
    }

    void transformRC(double x, double y, double z, double roll, double pitch, double yaw, cv::Mat& Trans, cv::Mat& R, cv::Mat& Transf) {
        cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
            1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll));

        cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
            cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch));

        cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
            cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1);

        R = Rz * Rx;

        Trans = (cv::Mat_<double>(3, 1) << x, y, z);

        Transf = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(Transf(cv::Rect(0, 0, 3, 3)));
        Trans.copyTo(Transf(cv::Rect(3, 0, 1, 3)));
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::CvBridgeException& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        int marker_id = 1;
        double marker_length = 0.05; // 0.03 for reality

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL); // for simulated box
        // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50); // for real box

        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            1396.8086675255468, 0.0, 960.0,
            0.0, 1396.8086675255468, 540.0,
            0.0, 0.0, 1.0);

        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(frame, dictionary, marker_corners, marker_ids);

        if (!marker_ids.empty()) {
            for (size_t i = 0; i < marker_ids.size(); ++i) {
                if (i == 1) {
                    std::vector<cv::Vec3d> rvecs, tvecs;
                    cv::aruco::estimatePoseSingleMarkers(marker_corners[i], marker_length, camera_matrix, dist_coeffs, rvecs, tvecs);
                    cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids, cv::Scalar(0, 255, 0));
                    cv::aruco::drawAxis(frame, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.05);

                    cv::Mat R;
                    cv::Rodrigues(rvecs[0], R);

                    cv::Mat Trans = (cv::Mat_<double>(3, 1) << tvecs[0][0], tvecs[0][1], tvecs[0][2]);
                    cv::Mat Transf = cv::Mat::eye(4, 4, CV_64F);
                    R.copyTo(Transf(cv::Rect(0, 0, 3, 3)));
                    Trans.copyTo(Transf(cv::Rect(3, 0, 1, 3)));

                    cv::Mat Trans_r_c, rot_r_c, Transf_r_c;
                    transformRC(camera[0], camera[1], camera[2], camera[3], camera[4], camera[5], Trans_r_c, rot_r_c, Transf_r_c);

                    cv::Mat Transf_r = Transf_r_c * Transf;

                    double x = Transf_r.at<double>(0, 3) + 0.025;
                    double y = Transf_r.at<double>(1, 3);
                    double z = -0.25;

                    std_msgs::Float64MultiArray point_msg;
                    point_msg.data = {x, y, z, 0.0};
                    image_pub.publish(point_msg);

                    ros::shutdown();
                }
            }
        }

        cv::namedWindow("Camera", cv::WINDOW_NORMAL);
        cv::resizeWindow("Camera", 800, 600);
        cv::moveWindow("Camera", 0, 0);
        cv::imshow("Camera", frame);
        cv::waitKey(1);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher image_pub;
    ros::Subscriber image_sub;
    ros::Subscriber camera_param_sub;
    ros::Rate rate;

    std::vector<double> camera;
    bool marker_detected;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_sub_d");

    try {
        CameraImage camera_image;
        ros::spin();
    } catch (const ros::Exception& e) {
        ROS_ERROR("ROS exception: %s", e.what());
    }

    return 0;
}
