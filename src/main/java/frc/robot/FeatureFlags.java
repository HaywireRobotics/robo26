// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.data.VisionEstimationStrategy;

/** Feature flags to control features of the robot
 * 
 */
public class FeatureFlags {
    // Feature Flags

    // Enable tuning of the field forward using SysId
    public static final boolean kEnableFeedforwardTuning = false; // A, B, X, Y run feedforward tuning code for the Sysid tool
    public static final boolean kEnableShooterTuning = false; // A
    // Disables setting motor voltage at the SwerveModule level
    public static final boolean kEnableDriving = true;
    // Disabled the intake assembly motor.
    public static final boolean kEnableIntakeAssemblyMotion = false;

    // Use the provided SwerveDrivePoseEstimator instead of SwerveDriveOdometry in pose estimation calculations
    public static final VisionEstimationStrategy kVisionEstimationStrategy = VisionEstimationStrategy.kMultiTag;



    public static void verify() {
        
    }
}
