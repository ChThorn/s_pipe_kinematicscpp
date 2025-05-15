#include "forward_kinematics.h"
#include <iostream>

namespace s_pipe {

ForwardKinematics::ForwardKinematics(const std::vector<DHParam>& dh_params)
    : dh_params_(dh_params),
      joint_indices_(getJointIndices()),
      joint_offsets_(getJointOffsets()) {
}

Eigen::Matrix4d ForwardKinematics::calculateTransformMatrix(const DHParam& dh) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    
    double ct = cos(dh.theta);
    double st = sin(dh.theta);
    double ca = cos(dh.alpha);
    double sa = sin(dh.alpha);
    
    matrix(0, 0) = ct;
    matrix(0, 1) = -st * ca;
    matrix(0, 2) = st * sa;
    matrix(0, 3) = dh.a * ct;
    
    matrix(1, 0) = st;
    matrix(1, 1) = ct * ca;
    matrix(1, 2) = -ct * sa;
    matrix(1, 3) = dh.a * st;
    
    matrix(2, 0) = 0;
    matrix(2, 1) = sa;
    matrix(2, 2) = ca;
    matrix(2, 3) = dh.d;
    
    matrix(3, 0) = 0;
    matrix(3, 1) = 0;
    matrix(3, 2) = 0;
    matrix(3, 3) = 1;
    
    return matrix;
}

Eigen::Matrix4d ForwardKinematics::compute(const std::vector<double>& joint_angles) {
    if (joint_angles.size() != 6) {
        throw std::invalid_argument("Joint angles vector must have 6 elements");
    }
    
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    
    for (size_t i = 0; i < dh_params_.size(); ++i) {
        DHParam currentDH = dh_params_[i];
        
        // Update theta with joint angle if it's a rotational joint
        for (int j = 0; j < 6; ++j) {
            if (i == joint_indices_[j]) {
                currentDH.theta = joint_angles[j] + joint_offsets_[j];
                break;
            }
        }
        
        Eigen::Matrix4d currentTransform = calculateTransformMatrix(currentDH);
        result = result * currentTransform;
    }
    
    return result;
}

Eigen::Matrix4d ForwardKinematics::computeDeg(const std::vector<double>& joint_angles_deg) {
    std::vector<double> joint_angles_rad(joint_angles_deg.size());
    for (size_t i = 0; i < joint_angles_deg.size(); ++i) {
        joint_angles_rad[i] = deg2rad(joint_angles_deg[i]);
    }
    return compute(joint_angles_rad);
}

std::vector<Eigen::Matrix4d> ForwardKinematics::computeJointTransforms(const std::vector<double>& joint_angles) {
    if (joint_angles.size() != 6) {
        throw std::invalid_argument("Joint angles vector must have 6 elements");
    }
    
    std::vector<Eigen::Matrix4d> transforms;
    Eigen::Matrix4d currentTransform = Eigen::Matrix4d::Identity();
    transforms.push_back(currentTransform); // Base frame
    
    for (size_t i = 0; i < dh_params_.size(); ++i) {
        DHParam currentDH = dh_params_[i];
        
        // Update theta with joint angle if it's a rotational joint
        for (int j = 0; j < 6; ++j) {
            if (i == joint_indices_[j]) {
                currentDH.theta = joint_angles[j] + joint_offsets_[j];
                break;
            }
        }
        
        Eigen::Matrix4d localTransform = calculateTransformMatrix(currentDH);
        currentTransform = currentTransform * localTransform;
        transforms.push_back(currentTransform);
    }
    
    return transforms;
}

} // namespace s_pipe