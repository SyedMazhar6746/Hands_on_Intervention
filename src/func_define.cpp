#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

// Define the transformation matrix type
typedef Eigen::Matrix<double, 4, 4> TransformMatrix;

// Function to provide rotation along one axis as a transformation matrix
TransformMatrix rot(const char axis, const double theta) {
    TransformMatrix matrix = TransformMatrix::Identity();
    if (axis == 'x') {
        matrix.block<3, 3>(0, 0) = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX()).toRotationMatrix();
    } else if (axis == 'z') {
        matrix.block<3, 3>(0, 0) = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    } else {
        throw std::invalid_argument("Invalid axis. Must be 'x', or 'z'.");
    }
    return matrix;
}

// Function to provide translation as a transformation matrix
TransformMatrix transl(const Eigen::Vector3d& translation) {
    if (translation.size() != 3) {
        throw std::invalid_argument("Invalid translation vector. Must have three elements.");
    }
    TransformMatrix matrix = TransformMatrix::Identity();
    matrix.block<3, 1>(0, 3) = translation;
    return matrix;
}

// Function to find the weighted DLS (i.e., to find Jacobian inverse)
Eigen::MatrixXd W_DLS(const Eigen::MatrixXd& A, const double damping, const Eigen::VectorXd& weight) {
    Eigen::MatrixXd w = weight.asDiagonal();
    Eigen::MatrixXd w_i = w.inverse();

    Eigen::MatrixXd A_damped = A * w_i * A.transpose() + damping * damping * Eigen::MatrixXd::Identity(A.rows(), A.rows());
    Eigen::MatrixXd A_damped_inv = A_damped.inverse();
    Eigen::MatrixXd A_DLS = w_i * A.transpose() * A_damped_inv;

    return A_DLS;
}

// Function to provide the transformation from base_footprint to swiftpro_base_link
TransformMatrix fixed_transform() {
    double angle = -M_PI / 2.0;
    TransformMatrix T_bf_sbl;
    T_bf_sbl << std::cos(angle), -std::sin(angle), 0, 0.051,
                std::sin(angle), std::cos(angle), 0, 0,
                0, 0, 1, -0.198,
                0, 0, 0, 1;
    return T_bf_sbl;
}

// Function to evaluate the kinematics of the robot
std::vector<TransformMatrix> kinematics_tb(const Eigen::VectorXd& theta, const TransformMatrix& Tb) {
    std::vector<TransformMatrix> T;
    T.push_back(Tb);

    TransformMatrix T_bf_sbl = fixed_transform();

    TransformMatrix T1 = rot('z', theta(0)) * transl(Eigen::Vector3d(0.0132, 0, 0)) * rot('x', -M_PI / 2.0) * transl(Eigen::Vector3d(0, 0.108, 0));
    TransformMatrix T2 = transl(Eigen::Vector3d(-0.142 * std::sin(theta(1)), 0.142 * std::cos(theta(1)), 0));
    TransformMatrix T3 = transl(Eigen::Vector3d(0.1588 * std::cos(theta(2)), 0.1588 * std::sin(theta(2)), 0)) * rot('x', M_PI / 2.0) * transl(Eigen::Vector3d(0.056, 0, 0));
    TransformMatrix T4 = rot('z', theta(3)) * transl(Eigen::Vector3d(0, 0, 0.0722));

    std::vector<TransformMatrix> T_l = { T_bf_sbl, T1, T2, T3, T4 };

    for (size_t i = 0; i < T_l.size(); ++i) {
        TransformMatrix t = T.back() * T_l[i];
        T.push_back(t);
    }

    return T;
}

// Function to compute the Jacobian matrix
Eigen::MatrixXd Jacobian(const Eigen::VectorXd& theta, const double yaw, const double d, const int link) {
    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);

    Eigen::MatrixXd Final1(6, 1);
    Final1 << -d * std::sin(yaw) + (-std::sin(theta(0)) * std::sin(yaw) + std::cos(theta(0)) * std::cos(yaw)) * (0.1588 * std::cos(theta(2)) + 0.056) - 0.142 * (-std::sin(theta(0)) * std::sin(yaw) + std::cos(theta(0)) * std::cos(yaw)) * std::sin(theta(1)) + 0.0132 * std::sin(theta(0)) * std::sin(yaw) - 0.051 * std::sin(yaw) + 0.0132 * std::cos(theta(0)) * std::cos(yaw),
              d * std::cos(yaw) + (std::sin(theta(0)) * std::cos(yaw) + std::sin(yaw) * std::cos(theta(0))) * (0.1588 * std::cos(theta(2)) + 0.056) - 0.142 * (std::sin(theta(0)) * std::cos(yaw) + std::sin(yaw) * std::cos(theta(0))) * std::sin(theta(1)) + 0.0132 * std::sin(theta(0)) * std::cos(yaw) + 0.0132 * std::sin(yaw) * std::cos(theta(0)) + 0.051 * std::cos(yaw),
              0, 0, 0, 1;

    Eigen::MatrixXd Final2(6, 1);
    Final2 << std::cos(yaw),
              std::sin(yaw),
              0,
              0,
              0,
              0;

    Eigen::MatrixXd Final3(6, 1);
    Final3 << (-std::sin(theta(0)) * std::sin(yaw) + std::cos(theta(0)) * std::cos(yaw)) * (0.1588 * std::cos(theta(2)) + 0.056) - 0.142 * (-std::sin(theta(0)) * std::sin(yaw) + std::cos(theta(0)) * std::cos(yaw)) * std::sin(theta(1)) - 0.0132 * std::sin(theta(0)) * std::sin(yaw) + 0.0132 * std::cos(theta(0)) * std::cos(yaw),
              (std::sin(theta(0)) * std::cos(yaw) + std::sin(yaw) * std::cos(theta(0))) * (0.1588 * std::cos(theta(2)) + 0.056) - 0.142 * (std::sin(theta(0)) * std::cos(yaw) + std::sin(yaw) * std::cos(theta(0))) * std::sin(theta(1)) + 0.0132 * std::sin(theta(0)) * std::cos(yaw) + 0.0132 * std::sin(yaw) * std::cos(theta(0)),
              0,
              0,
              0,
              1;

    Eigen::MatrixXd Final4(6, 1);
    Final4 << -0.142 * (std::sin(theta(0)) * std::cos(yaw) + std::sin(yaw) * std::cos(theta(0))) * std::cos(theta(1)),
              -0.142 * (std::sin(theta(0)) * std::sin(yaw) - std::cos(theta(0)) * std::cos(yaw)) * std::cos(theta(1)),
              0.142 * std::sin(theta(1)),
              0,
              0,
              0;

    Eigen::MatrixXd Final5(6, 1);
    Final5 << -0.1588 * (std::sin(theta(0)) * std::cos(yaw) + std::sin(yaw) * std::cos(theta(0))) * std::sin(theta(2)),
              -0.1588 * (std::sin(theta(0)) * std::sin(yaw) - std::cos(theta(0)) * std::cos(yaw)) * std::sin(theta(2)),
              -0.1588 * std::cos(theta(2)),
              0,
              0,
              0;

    Eigen::MatrixXd Final6(6, 1);
    Final6 << 0,
              0,
              0,
              0,
              0,
              1;

    std::vector<Eigen::MatrixXd> finals = { Final1, Final2, Final3, Final4, Final5, Final6 };

    for (int i = 0; i < finals.size(); ++i) {
        J.col(link + i) = finals[i];
    }

    return J;
}

// Function to convert transformation matrix to PoseStamped message
geometry_msgs::PoseStamped pose_EE(const TransformMatrix& transformation_matrix) {
    Eigen::Vector3d t = transformation_matrix.block<3, 1>(0, 3);
    Eigen::Quaterniond q(transformation_matrix.block<3, 3>(0, 0));

    geometry_msgs::PoseStamped p;
    p.header.frame_id = "world";
    p.header.stamp = ros::Time::now();

    p.pose.position.x = t(0);
    p.pose.position.y = t(1);
    p.pose.position.z = t(2);

    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    return p;
}

// Function to generate goal pose
geometry_msgs::PoseStamped goal_pose(const Eigen::Vector3d& trans, const double angle) {
    Eigen::Quaterniond quat = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());

    geometry_msgs::PoseStamped g_pose;
    g_pose.header.frame_id = "world";
    g_pose.pose.position.x = trans(0);
    g_pose.pose.position.y = trans(1);
    g_pose.pose.position.z = trans(2);

    g_pose.pose.orientation.x = quat.x();
    g_pose.pose.orientation.y = quat.y();
    g_pose.pose.orientation.z = quat.z();
    g_pose.pose.orientation.w = quat.w();

    return g_pose;
}

// Function to scale the elements of a vector except the specified index
Eigen::VectorXd scale(const Eigen::VectorXd& dq, const double x, const int j) {
    Eigen::VectorXd scaled_dq = Eigen::VectorXd::Zero(dq.size());
    for (int i = 0, k = 0; i < dq.size(); ++i) {
        if (i == j) {
            scaled_dq(i) = x;
        } else {
            scaled_dq(i) = (x * dq(k)) / dq(j);
            ++k;
        }
    }
    return scaled_dq;
}
