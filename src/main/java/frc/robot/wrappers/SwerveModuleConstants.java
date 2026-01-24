// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

/** The constants for a SwerveModule. Includes the CAN ID of the turn and drive motor, and the encoder, 
 * and the offset. */
public class SwerveModuleConstants {
    public final int turnMotor;
    public final int driveMotor;
    public final double offset;
    public final int encoder;
    public SwerveModuleConstants(int turnMotorId, int driveMotorId, int encoderId, double offsetvalue) {
        turnMotor = turnMotorId;
        driveMotor = driveMotorId;
        offset = offsetvalue;
        encoder = encoderId;

    }
}
