package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.wrappers.SwerveModuleConstants;

public class Constants {
    public static final double kInchesToMeters = 0.0254;

    public static final SparkBaseConfig kNeoNominalConfig = new SparkMaxConfig().smartCurrentLimit(80).idleMode(IdleMode.kBrake).inverted(false);
    public static final SparkBaseConfig kNeo550NominalConfig = new SparkMaxConfig().smartCurrentLimit(20).idleMode(IdleMode.kBrake).inverted(false);

    public static final int kTestMotorId = 5;
    public static final double kTestMotorSpeed = 5.0;

    public static final double kDriveTrainWidth = 17.5 * kInchesToMeters;
    public static final double kDriveTrainLength = 17 * kInchesToMeters;

    public static final double kRobotWidth = (33.0 + (3.0/32.0)) * kInchesToMeters;
    public static final double kRobotLength = 38.5 * kInchesToMeters;

    // Swerves
    public static final double kDriveGearRatio = 6.75;
    public static final double kWheelDiameter = 4.00000;
    public static final double kSlowModeDivider = 1.5;
    public static final double kNavigationMultiplier = 1;
    public static final double kRotationMultiplier = 5;

    public static final double kMaxAngularVelocity = 10;
    public static final double kMaxAngularAcceleration = 20;
    public static final SwerveModuleConstants kFrontRightSwerve = new SwerveModuleConstants(
        4,
        6,
        8,
        -0.304931640625
    );
    public static final SwerveModuleConstants kFrontLeftSwerve = new SwerveModuleConstants(
        14,
        20,
        2,
        -0.772216796875
    );
    public static final SwerveModuleConstants kBackRightSwerve = new SwerveModuleConstants(
        12,
        3,
        5,
        -0.058837890625
    );
    public static final SwerveModuleConstants kBackLeftSwerve = new SwerveModuleConstants(
        1,
        9,
        11,
        -0.28857421875
    );
    public static final double kSwerveDriveKP = 0.44072;
    public static final double kSwerveDriveKI = 0.0;
    public static final double kSwerveDriveKD = 0.0;
    public static final double kSwerveDriveKS = 0.098939;
    public static final double kSwerveDriveKV = 2.1788;
    public static final double kSwerveDriveKA = 0.39833;

    public static final double kSwerveTurningKP = 5;
    public static final double kSwerveTurningKI = 0;
    public static final double kSwerveTurningKD = 0;
    public static final double kSwerveTurningKS = 0.2;
    public static final double kSwerveTurningKV = 47.12;

    public static final double kMaxPoseAmbiguity = 0.1; // Ambiguity Ratio
    public static final double kMaxTagDistance = 1.5; // Meters
}
