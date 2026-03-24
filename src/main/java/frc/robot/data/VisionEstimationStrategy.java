// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

/** Represents an estimation strategy for the Camera module. */
public enum VisionEstimationStrategy {
    /**
     * Use single tag pose estimation with interpolation based on april tag ambigutiy.
     */
    kSingleTag,
    /**
     * Use single tag pose estimation with the builtin Kalman Filter.
     */
    kSingleTagFiltered,
    /**
     * Use multi tag pose estimation. The field data on the camera must be updated.
     */
    kMultiTag
}
