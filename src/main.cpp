// #include "s_pipe_robot.h"
// #include "forward_kinematics.h"
// #include "inverse_kinematics.h"
// #include <iostream>
// #include <iomanip>
// #include <vector>
// #include <Eigen/Dense>

// using namespace s_pipe;

// // Function to print a 4x4 matrix in a readable format
// void printMatrix(const Eigen::Matrix4d& matrix) {
//     std::cout << std::fixed << std::setprecision(6);
//     for (int i = 0; i < 4; ++i) {
//         for (int j = 0; j < 4; ++j) {
//             std::cout << matrix(i, j) << "\t";
//         }
//         std::cout << std::endl;
//     }
// }

// // Calculate position error
// double calculatePositionError(const Eigen::Matrix4d& target, const Eigen::Matrix4d& actual) {
//     Eigen::Vector3d target_pos = target.block<3, 1>(0, 3);
//     Eigen::Vector3d actual_pos = actual.block<3, 1>(0, 3);
//     return (target_pos - actual_pos).norm();
// }

// // Calculate orientation error
// double calculateOrientationError(const Eigen::Matrix4d& target, const Eigen::Matrix4d& actual) {
//     Eigen::Matrix3d target_rot = target.block<3, 3>(0, 0);
//     Eigen::Matrix3d actual_rot = actual.block<3, 3>(0, 0);
//     Eigen::Matrix3d rot_error = target_rot * actual_rot.transpose();
//     Eigen::AngleAxisd angle_axis(rot_error);
//     return angle_axis.angle();
// }

// int main() {
//     // Create forward and inverse kinematics objects
//     ForwardKinematics fk;
//     InverseKinematics ik;
    
//     // Test joint angles in degrees
//     std::vector<double> test_angles_deg = {30.0, 45.0, -30.0, 60.0, -45.0, 90.0};
    
//     std::cout << "\nOriginal joint angles in degrees: [";
//     for (size_t i = 0; i < test_angles_deg.size(); ++i) {
//         std::cout << test_angles_deg[i];
//         if (i < test_angles_deg.size() - 1) std::cout << ", ";
//     }
//     std::cout << "]" << std::endl;
    
//     // Calculate forward kinematics
//     Eigen::Matrix4d target_pose = fk.computeDeg(test_angles_deg);
    
//     std::cout << "\nTarget End Effector Pose from forward kinematics:" << std::endl;
//     printMatrix(target_pose);
    
//     std::cout << "\nSolving inverse kinematics for perfect match..." << std::endl;
    
//     // Initial guess (neutral position)
//     std::vector<double> initial_guess_deg = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
//     // Solve inverse kinematics
//     std::vector<double> ik_solution_deg = ik.computeDeg(
//         target_pose, initial_guess_deg, 0.1, 0.000001, 1000);
    
//     // Print IK solution
//     std::cout << "\nInverse Kinematics Solution (in degrees): [";
//     for (size_t i = 0; i < ik_solution_deg.size(); ++i) {
//         std::cout << ik_solution_deg[i];
//         if (i < ik_solution_deg.size() - 1) std::cout << ", ";
//     }
//     std::cout << "]" << std::endl;
    
//     // Verify the solution using forward kinematics
//     Eigen::Matrix4d verified_pose = fk.computeDeg(ik_solution_deg);
    
//     std::cout << "\nVerified End Effector Pose from IK solution:" << std::endl;
//     printMatrix(verified_pose);
    
//     // Calculate errors
//     double pos_error = calculatePositionError(target_pose, verified_pose);
//     double ori_error = rad2deg(calculateOrientationError(target_pose, verified_pose));
    
//     std::cout << "\nPosition Error: " << pos_error << " mm" << std::endl;
//     std::cout << "Orientation Error: " << ori_error << " degrees" << std::endl;
    
//     // Compare original angles with IK solution
//     std::cout << "\nComparison of Original vs. IK Solution:" << std::endl;
//     std::cout << "Joint | Original | IK Solution | Difference" << std::endl;
//     std::cout << "------|----------|------------|------------" << std::endl;
    
//     double total_diff = 0.0;
//     for (size_t i = 0; i < test_angles_deg.size(); ++i) {
//         double orig = test_angles_deg[i];
//         double ik = ik_solution_deg[i];
//         double diff = std::abs(orig - ik);
//         // Normalize angle difference
//         if (diff > 180.0) diff = 360.0 - diff;
        
//         total_diff += diff;
        
//         std::cout << std::fixed << std::setprecision(6);
//         std::cout << "  " << (i+1) << "  | " 
//                   << std::setw(8) << orig << " | " 
//                   << std::setw(10) << ik << " | " 
//                   << std::setw(10) << diff << std::endl;
//     }
    
//     double avg_diff = total_diff / test_angles_deg.size();
//     std::cout << "\nAverage joint angle difference: " << avg_diff << " degrees" << std::endl;
    
//     // Test with perfect recovery (using original angles as initial guess)
//     std::cout << "\n\nTesting perfect recovery with original angles as initial guess..." << std::endl;
    
//     std::vector<double> perfect_recovery_deg = ik.computeDeg(
//         target_pose, test_angles_deg, 0.1, 0.000001, 1000);
    
//     // Print perfect recovery solution
//     std::cout << "\nPerfect Recovery Solution (in degrees): [";
//     for (size_t i = 0; i < perfect_recovery_deg.size(); ++i) {
//         std::cout << perfect_recovery_deg[i];
//         if (i < perfect_recovery_deg.size() - 1) std::cout << ", ";
//     }
//     std::cout << "]" << std::endl;
    
//     // Calculate and print recovery differences
//     std::cout << "\nComparison of Original vs. Perfect Recovery:" << std::endl;
//     std::cout << "Joint | Original | Recovery | Difference" << std::endl;
//     std::cout << "------|----------|----------|------------" << std::endl;
    
//     double total_recovery_diff = 0.0;
//     for (size_t i = 0; i < test_angles_deg.size(); ++i) {
//         double orig = test_angles_deg[i];
//         double rec = perfect_recovery_deg[i];
//         double diff = std::abs(orig - rec);
//         // Normalize angle difference
//         if (diff > 180.0) diff = 360.0 - diff;
        
//         total_recovery_diff += diff;
        
//         std::cout << std::fixed << std::setprecision(10);
//         std::cout << "  " << (i+1) << "  | " 
//                   << std::setw(8) << orig << " | " 
//                   << std::setw(10) << rec << " | " 
//                   << std::setw(10) << diff << std::endl;
//     }
    
//     double avg_recovery_diff = total_recovery_diff / test_angles_deg.size();
//     std::cout << "\nAverage recovery difference: " << avg_recovery_diff << " degrees" << std::endl;
    
//     // Test with a new arbitrary target position
//     std::cout << "\n\nTesting with a new arbitrary target position..." << std::endl;
    
//     Eigen::Matrix4d new_target_pose = Eigen::Matrix4d::Identity();
//     new_target_pose(0, 3) = 300.0;  // X position
//     new_target_pose(1, 3) = 100.0;  // Y position
//     new_target_pose(2, 3) = 700.0;  // Z position
    
//     // Set a simple orientation (identity rotation)
//     std::cout << "\nNew Target Pose:" << std::endl;
//     printMatrix(new_target_pose);
    
//     // Solve inverse kinematics for new target
//     std::vector<double> new_ik_solution_deg = ik.computeDeg(
//         new_target_pose, initial_guess_deg, 0.1, 0.000001, 1000);
    
//     // Print new IK solution
//     std::cout << "\nNew Inverse Kinematics Solution (in degrees): [";
//     for (size_t i = 0; i < new_ik_solution_deg.size(); ++i) {
//         std::cout << new_ik_solution_deg[i];
//         if (i < new_ik_solution_deg.size() - 1) std::cout << ", ";
//     }
//     std::cout << "]" << std::endl;
    
//     // Verify the new solution using forward kinematics
//     Eigen::Matrix4d new_verified_pose = fk.computeDeg(new_ik_solution_deg);
    
//     std::cout << "\nNew Verified End Effector Pose:" << std::endl;
//     printMatrix(new_verified_pose);
    
//     // Calculate errors for new position
//     double new_pos_error = calculatePositionError(new_target_pose, new_verified_pose);
//     double new_ori_error = rad2deg(calculateOrientationError(new_target_pose, new_verified_pose));
    
//     std::cout << "\nNew Position Error: " << new_pos_error << " mm" << std::endl;
//     std::cout << "New Orientation Error: " << new_ori_error << " degrees" << std::endl;
    
//     return 0;
// }


// #include "s_pipe_robot.h"
// #include "forward_kinematics.h"
// #include "inverse_kinematics.h"
// #include <iostream>
// #include <iomanip>
// #include <vector>
// #include <Eigen/Dense>

// using namespace s_pipe;

// // Function to print a 4x4 matrix in a readable format
// void printMatrix(const Eigen::Matrix4d& matrix) {
//     std::cout << std::fixed << std::setprecision(6);
//     for (int i = 0; i < 4; ++i) {
//         for (int j = 0; j < 4; ++j) {
//             std::cout << matrix(i, j) << "\t";
//         }
//         std::cout << std::endl;
//     }
// }

// // Calculate position error
// double calculatePositionError(const Eigen::Matrix4d& target, const Eigen::Matrix4d& actual) {
//     Eigen::Vector3d target_pos = target.block<3, 1>(0, 3);
//     Eigen::Vector3d actual_pos = actual.block<3, 1>(0, 3);
//     return (target_pos - actual_pos).norm();
// }

// // Calculate orientation error
// double calculateOrientationError(const Eigen::Matrix4d& target, const Eigen::Matrix4d& actual) {
//     Eigen::Matrix3d target_rot = target.block<3, 3>(0, 0);
//     Eigen::Matrix3d actual_rot = actual.block<3, 3>(0, 0);
//     Eigen::Matrix3d rot_error = target_rot * actual_rot.transpose();
//     Eigen::AngleAxisd angle_axis(rot_error);
//     return angle_axis.angle();
// }

// // Test function for a single configuration
// void testConfiguration(ForwardKinematics& fk, InverseKinematics& ik, 
//                       const std::vector<double>& joint_angles_deg,
//                       const std::string& test_name) {
//     std::cout << "\n=== TEST: " << test_name << " ===" << std::endl;
    
//     std::cout << "Input joint angles in degrees: [";
//     for (size_t i = 0; i < joint_angles_deg.size(); ++i) {
//         std::cout << joint_angles_deg[i];
//         if (i < joint_angles_deg.size() - 1) std::cout << ", ";
//     }
//     std::cout << "]" << std::endl;
    
//     // Calculate forward kinematics
//     Eigen::Matrix4d target_pose = fk.computeDeg(joint_angles_deg);
    
//     std::cout << "\nForward Kinematics Result:" << std::endl;
//     printMatrix(target_pose);
    
//     // Initialize with zero angles for a challenging test
//     std::vector<double> initial_guess_deg = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
//     // Solve inverse kinematics
//     std::vector<double> ik_solution_deg = ik.computeDeg(
//         target_pose, initial_guess_deg, 0.1, 0.000001, 1000);
    
//     // Print IK solution
//     std::cout << "\nInverse Kinematics Solution (in degrees): [";
//     for (size_t i = 0; i < ik_solution_deg.size(); ++i) {
//         std::cout << ik_solution_deg[i];
//         if (i < ik_solution_deg.size() - 1) std::cout << ", ";
//     }
//     std::cout << "]" << std::endl;
    
//     // Verify the solution using forward kinematics
//     Eigen::Matrix4d verified_pose = fk.computeDeg(ik_solution_deg);
    
//     std::cout << "\nFK Verification of IK Solution:" << std::endl;
//     printMatrix(verified_pose);
    
//     // Calculate errors
//     double pos_error = calculatePositionError(target_pose, verified_pose);
//     double ori_error = rad2deg(calculateOrientationError(target_pose, verified_pose));
    
//     std::cout << "\nPosition Error: " << pos_error << " mm" << std::endl;
//     std::cout << "Orientation Error: " << ori_error << " degrees" << std::endl;
    
//     // Print joint angle differences
//     std::cout << "\nJoint Differences:" << std::endl;
//     double total_diff = 0.0;
//     for (size_t i = 0; i < joint_angles_deg.size(); ++i) {
//         double orig = joint_angles_deg[i];
//         double ik = ik_solution_deg[i];
        
//         // Normalize angles for comparison (handle wrap-around)
//         while (orig > 180.0) orig -= 360.0;
//         while (orig < -180.0) orig += 360.0;
//         while (ik > 180.0) ik -= 360.0;
//         while (ik < -180.0) ik += 360.0;
        
//         double diff = std::abs(orig - ik);
//         if (diff > 180.0) diff = 360.0 - diff;
        
//         total_diff += diff;
//         std::cout << "Joint " << (i+1) << ": " << diff << " degrees" << std::endl;
//     }
    
//     std::cout << "Average joint difference: " << (total_diff / 6.0) << " degrees" << std::endl;


// }

// int main() {
//     // Create forward and inverse kinematics objects
//     ForwardKinematics fk;
//     InverseKinematics ik;
    
//     // Test 1: Standard configuration (your original test)
//     // std::vector<double> test1 = {30.0, 45.0, -30.0, 60.0, -45.0, 90.0};
//     std::vector<double> test1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//     testConfiguration(fk, ik, test1, "Standard Configuration");
    
//     // Test 2: Extended arm configuration
//     std::vector<double> test2 = {0.0, 90.0, 0.0, 0.0, 0.0, 0.0};
//     testConfiguration(fk, ik, test2, "Extended Arm");
    
//     // Test 3: Complex twist configuration
//     std::vector<double> test3 = {45.0, 30.0, -60.0, 120.0, 30.0, -90.0};
//     testConfiguration(fk, ik, test3, "Complex Twist");
    
//     // Test 4: Extreme angles configuration
//     std::vector<double> test4 = {175.0, -175.0, 175.0, -175.0, 175.0, -175.0};
//     testConfiguration(fk, ik, test4, "Extreme Angles");
    
//     // Test 5: Near-singular configuration (wrist aligned with base)
//     std::vector<double> test5 = {0.0, 0.0, -90.0, 0.0, 0.0, 0.0};
//     testConfiguration(fk, ik, test5, "Near-Singular Configuration");
    
//     return 0;
// }


#include "s_pipe_robot.h"
#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>

using namespace s_pipe;

// Function to print a 4x4 matrix in a readable format
void printMatrix(const Eigen::Matrix4d& matrix) {
    std::cout << std::fixed << std::setprecision(6);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << matrix(i, j) << "\t";
        }
        std::cout << std::endl;
    }
}

// Extract position from transformation matrix
Eigen::Vector3d getPosition(const Eigen::Matrix4d& pose) {
    return pose.block<3, 1>(0, 3);
}

// Extract Euler angles from rotation matrix
Eigen::Vector3d getEulerAngles(const Eigen::Matrix4d& pose) {
    Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
    
    // Extract Euler angles (roll, pitch, yaw) in ZYX order
    Eigen::Vector3d euler = rotation.eulerAngles(2, 1, 0);
    
    // Convert to degrees
    euler[0] = rad2deg(euler[0]); // Yaw (Z)
    euler[1] = rad2deg(euler[1]); // Pitch (Y)
    euler[2] = rad2deg(euler[2]); // Roll (X)
    
    return euler;
}

// Calculate position error
double calculatePositionError(const Eigen::Matrix4d& target, const Eigen::Matrix4d& actual) {
    Eigen::Vector3d target_pos = getPosition(target);
    Eigen::Vector3d actual_pos = getPosition(actual);
    return (target_pos - actual_pos).norm();
}

// Calculate orientation error
double calculateOrientationError(const Eigen::Matrix4d& target, const Eigen::Matrix4d& actual) {
    Eigen::Matrix3d target_rot = target.block<3, 3>(0, 0);
    Eigen::Matrix3d actual_rot = actual.block<3, 3>(0, 0);
    Eigen::Matrix3d rot_error = target_rot * actual_rot.transpose();
    Eigen::AngleAxisd angle_axis(rot_error);
    return angle_axis.angle();
}

// Test function for a single configuration
void testConfiguration(ForwardKinematics& fk, InverseKinematics& ik, 
                      const std::vector<double>& joint_angles_deg,
                      const std::string& test_name) {
    std::cout << "\n=== TEST: " << test_name << " ===" << std::endl;
    
    std::cout << "Input joint angles in degrees: [";
    for (size_t i = 0; i < joint_angles_deg.size(); ++i) {
        std::cout << joint_angles_deg[i];
        if (i < joint_angles_deg.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    // Calculate forward kinematics
    Eigen::Matrix4d target_pose = fk.computeDeg(joint_angles_deg);
    
    // Extract and print position and orientation
    Eigen::Vector3d target_position = getPosition(target_pose);
    Eigen::Vector3d target_orientation = getEulerAngles(target_pose);
    
    std::cout << "\nTarget End-Effector Position [mm]: [" 
              << target_position[0] << ", " 
              << target_position[1] << ", " 
              << target_position[2] << "]" << std::endl;
              
    std::cout << "Target End-Effector Orientation [deg]: [" 
              << target_orientation[2] << ", " // Roll (X)
              << target_orientation[1] << ", " // Pitch (Y)
              << target_orientation[0] << "]"  // Yaw (Z)
              << " (Roll, Pitch, Yaw)" << std::endl;
    
    std::cout << "\nForward Kinematics Result:" << std::endl;
    printMatrix(target_pose);
    
    // Initialize with zero angles for a challenging test
    std::vector<double> initial_guess_deg = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Solve inverse kinematics
    std::vector<double> ik_solution_deg = ik.computeDeg(
        target_pose, initial_guess_deg, 0.1, 0.000001, 1000);
    
    // Print IK solution
    std::cout << "\nInverse Kinematics Solution (in degrees): [";
    for (size_t i = 0; i < ik_solution_deg.size(); ++i) {
        std::cout << ik_solution_deg[i];
        if (i < ik_solution_deg.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    // Verify the solution using forward kinematics
    Eigen::Matrix4d verified_pose = fk.computeDeg(ik_solution_deg);
    
    // Extract and print position and orientation for verification
    Eigen::Vector3d verified_position = getPosition(verified_pose);
    Eigen::Vector3d verified_orientation = getEulerAngles(verified_pose);
    
    std::cout << "\nVerified End-Effector Position [mm]: [" 
              << verified_position[0] << ", " 
              << verified_position[1] << ", " 
              << verified_position[2] << "]" << std::endl;
              
    std::cout << "Verified End-Effector Orientation [deg]: [" 
              << verified_orientation[2] << ", " // Roll (X)
              << verified_orientation[1] << ", " // Pitch (Y)
              << verified_orientation[0] << "]"  // Yaw (Z)
              << " (Roll, Pitch, Yaw)" << std::endl;
    
    std::cout << "\nFK Verification of IK Solution:" << std::endl;
    printMatrix(verified_pose);
    
    // Calculate errors
    double pos_error = calculatePositionError(target_pose, verified_pose);
    double ori_error = rad2deg(calculateOrientationError(target_pose, verified_pose));
    
    std::cout << "\nPosition Error: " << pos_error << " mm" << std::endl;
    std::cout << "Orientation Error: " << ori_error << " degrees" << std::endl;
    
    // Print joint angle differences
    std::cout << "\nJoint Differences:" << std::endl;
    double total_diff = 0.0;
    for (size_t i = 0; i < joint_angles_deg.size(); ++i) {
        double orig = joint_angles_deg[i];
        double ik = ik_solution_deg[i];
        
        // Normalize angles for comparison (handle wrap-around)
        while (orig > 180.0) orig -= 360.0;
        while (orig < -180.0) orig += 360.0;
        while (ik > 180.0) ik -= 360.0;
        while (ik < -180.0) ik += 360.0;
        
        double diff = std::abs(orig - ik);
        if (diff > 180.0) diff = 360.0 - diff;
        
        total_diff += diff;
        std::cout << "Joint " << (i+1) << ": " << diff << " degrees" << std::endl;
    }
    
    std::cout << "Average joint difference: " << (total_diff / 6.0) << " degrees" << std::endl;
}

int main() {
    // Create forward and inverse kinematics objects
    ForwardKinematics fk;
    InverseKinematics ik;
    
    // Test 1: Standard configuration
    // std::vector<double> test1 = {30.0, 45.0, -30.0, 60.0, -45.0, 90.0};
    std::vector<double> test1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    testConfiguration(fk, ik, test1, "Standard Configuration");
    
    // Test 2: Extended arm configuration
    // std::vector<double> test2 = {0.0, 90.0, 0.0, 0.0, 0.0, 0.0};
    // testConfiguration(fk, ik, test2, "Extended Arm");
    
    // // Test 3: Complex twist configuration
    // std::vector<double> test3 = {45.0, 30.0, -60.0, 120.0, 30.0, -90.0};
    // testConfiguration(fk, ik, test3, "Complex Twist");
    
    // // Test 4: Extreme angles configuration
    // std::vector<double> test4 = {175.0, -175.0, 175.0, -175.0, 175.0, -175.0};
    // testConfiguration(fk, ik, test4, "Extreme Angles");
    
    // // Test 5: Near-singular configuration (wrist aligned with base)
    // std::vector<double> test5 = {0.0, 0.0, -90.0, 0.0, 0.0, 0.0};
    // testConfiguration(fk, ik, test5, "Near-Singular Configuration");
    
    return 0;
}