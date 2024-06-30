#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "func_define.hpp" // assuming function definitions are in this header

// Forward declarations
class Task;

class Manipulator {
private:
    Eigen::VectorXd theta;
    std::vector<bool> revolute;
    int dof;
    double r;
    Eigen::Vector3d eta;
    Eigen::MatrixXd T;

public:
    Manipulator(const Eigen::VectorXd& theta)
        : theta(theta), revolute({true, false, true, true, true, true}), dof(revolute.size()), r(0.0), eta(Eigen::Vector3d::Zero()), T(Eigen::MatrixXd::Zero(4, 4)) {}

    void update(const Eigen::MatrixXd& dq, double dt, const Eigen::VectorXd& state) {
        theta.tail(dq.rows() - 2) += dq.block(2, 0, dq.rows() - 2, 1) * dt;

        eta << state(0), state(1), state(2);

        Eigen::MatrixXd Tb(4, 4);
        Tb << std::cos(eta(2)), -std::sin(eta(2)), 0, eta(0),
              std::sin(eta(2)), std::cos(eta(2)), 0, eta(1),
              0, 0, 1, 0,
              0, 0, 0, 1;

        T = kinematics_tb(theta, Tb);
    }

    Eigen::MatrixXd getEEJacobian(int link) const {
        return Jacobian(theta, eta(2), eta(0), link);
    }

    Eigen::MatrixXd getEETransform() const {
        return T.block(0, 0, 4, 4);
    }

    double getJointPos(int joint) const {
        return theta(joint - 3);
    }

    Eigen::Vector3d getBasePose() const {
        return eta;
    }

    int getDOF() const {
        return dof;
    }

    Eigen::MatrixXd get_Se_LTransform(int link) const {
        return T.block((link - 1) * 4, 0, 4, 4);
    }
};

class Task {
protected:
    std::string name;
    Eigen::VectorXd sigma_d;
    Manipulator* mobi_base;
    bool active;
    double a;
    Eigen::MatrixXd J;
    Eigen::VectorXd err;

public:
    Task(const std::string& name, const Eigen::VectorXd& desired)
        : name(name), sigma_d(desired), mobi_base(nullptr), active(false), a(0), J(Eigen::MatrixXd::Zero(1, 1)), err(Eigen::VectorXd::Zero(1)) {}

    virtual ~Task() {}

    virtual void update(Manipulator* robot) = 0;

    bool bool_is_Active() const {
        return active;
    }

    void setDesired(const Eigen::VectorXd& value) {
        sigma_d = value;
    }

    Eigen::VectorXd getDesired() const {
        return sigma_d;
    }

    Eigen::MatrixXd getJacobian() const {
        return J;
    }

    Eigen::VectorXd getError() const {
        return err;
    }

    Eigen::Vector3d get_mobi_base() const {
        return mobi_base->getBasePose();
    }

    Eigen::MatrixXd get_eep() const {
        return mobi_base->getEETransform();
    }
};

class Position3D : public Task {
private:
    int link;

public:
    Position3D(const std::string& name, const Eigen::VectorXd& desired, int link)
        : Task(name, desired), link(link) {
        J.resize(3, link);
        err.resize(3);
        active = true;
    }

    void update(Manipulator* robot) override {
        J = robot->getEEJacobian(link).block(0, 0, 3, link);
        Eigen::MatrixXd X = robot->getEETransform().block(0, 3, 3, 1);
        err = (getDesired() - X).block(0, 0, 3, 1);
    }
};

class Orientation3D : public Task {
private:
    int link;

public:
    Orientation3D(const std::string& name, const Eigen::VectorXd& desired, int link)
        : Task(name, desired), link(link) {
        J.resize(1, 6);
        err.resize(1);
        active = true;
    }

    void update(Manipulator* robot) override {
        J = robot->getEEJacobian(link).block(5, 0, 1, 6);
        Eigen::MatrixXd Y = robot->getEETransform();
        double orien = std::atan2(Y(1, 0), Y(0, 0));
        err(0) = getDesired()(0) - orien;
    }
};

class BaseOrientation3D : public Task {
private:
    int link;

public:
    BaseOrientation3D(const std::string& name, const Eigen::VectorXd& desired, int link)
        : Task(name, desired), link(link) {
        J.resize(1, 6);
        err.resize(1);
        active = true;
    }

    void update(Manipulator* robot) override {
        J = robot->getEEJacobian(link).block(5, 0, 1, 6);
        Eigen::MatrixXd Y = robot->getEETransform();
        double orien = std::atan2(Y(1, 0), Y(0, 0));
        err(0) = getDesired()(0) - orien;
    }
};

class Configuration3D : public Task {
private:
    int link;

public:
    Configuration3D(const std::string& name, const Eigen::VectorXd& desired, int link)
        : Task(name, desired), link(link) {
        J.resize(4, link);
        err.resize(4);
        active = true;
    }

    void update(Manipulator* robot) override {
        J.block(0, 0, 3, link) = robot->getEEJacobian(link).block(0, 0, 3, link);
        J.row(3) = robot->getEEJacobian(link).row(5);
        Eigen::MatrixXd transf = robot->getEETransform();
        Eigen::VectorXd eep = transf.block(0, 3, 3, 1);
        double orien = std::atan2(transf(1, 0), transf(0, 0));
        err = getDesired() - Eigen::VectorXd::Vcat(eep, orien);
    }
};

class Jointlimits3D : public Task {
private:
    int link;
    Eigen::VectorXd activation;

public:
    Jointlimits3D(const std::string& name, const Eigen::VectorXd& desired, const Eigen::VectorXd& activation, int link)
        : Task(name, desired), activation(activation), link(link) {
        J.resize(1, 6);
        err.resize(1);
        a = 0;
    }

    void wrap_angle(double& angle) {
        angle = std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    void update(Manipulator* robot) override {
        J = robot->getEEJacobian(link).block(5, 0, 1, 6);
        Eigen::MatrixXd link_transform = robot->get_Se_LTransform(link);
        double orien = std::atan2(link_transform(1, 2), link_transform(0, 2));
        
        if (a == 1 && orien > activation(2)) {
            a = 0;
            active = false;
            err(0) = 0.0;
        }

        if (a == -1 && orien < activation(0)) {
            a = 0;
            active = false;
            err(0) = 0.0;
        }

        if (a == 0 && orien > activation(1)) {
            a = -1;
            active = true;
            err(0) = -1.0;
        }

        if (a == 0 && orien < activation(3)) {
            a = 1;
            active = true;
            err(0) = 1.0;
        }
    }
};

class JointPosition3D : public Task {
private:
    int link;

public:
    JointPosition3D(const std::string& name, const Eigen::VectorXd& desired, int link)
        : Task(name, desired), link(link) {
        J.resize(1, 6);
        err.resize(1);
        active = true;
    }

    void update(Manipulator* robot) override {
        J = robot->getEEJacobian(link).block(5, 0, 1, 6);
        double sigma = robot->getJointPos(link);
        err(0) = getDesired()(0) - sigma;
    }
};

