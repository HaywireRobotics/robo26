package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.data.SwerveModuleConstants;

public class Constants {
    public static final double kInchesToMeters = 0.0254;

    public static final SparkBaseConfig kNeoNominalConfig = new SparkMaxConfig().smartCurrentLimit(80).idleMode(IdleMode.kBrake).inverted(false);
    public static final SparkBaseConfig kNeoNoBreakConfig = new SparkMaxConfig().smartCurrentLimit(80).idleMode(IdleMode.kCoast).inverted(false);
    public static final SparkBaseConfig kNeo550NominalConfig = new SparkMaxConfig().smartCurrentLimit(20).idleMode(IdleMode.kBrake).inverted(false);

    public static final double kDriveTrainWidth = 17.5 * kInchesToMeters;
    public static final double kDriveTrainLength = 17 * kInchesToMeters;

    public static final double kRobotWidth = 27 * kInchesToMeters;
    public static final double kRobotLength = 27 * kInchesToMeters;

    // Swerves
    public static final double kDriveGearRatio = 6.75;
    public static final double kWheelDiameter = 4.00000;
    public static final double kSlowModeDivider = 1.5;
    public static final double kNavigationMultiplier = 1;
    public static final double kRotationMultiplier = 5;

    public static final double kMaxAngularVelocity = 10;
    public static final double kMaxAngularAcceleration = 20;
    public static final SwerveModuleConstants kFrontRightSwerve = new SwerveModuleConstants(
        7,
        6,
        14,
        0.11083984375 + 0.5
    );
    public static final SwerveModuleConstants kFrontLeftSwerve = new SwerveModuleConstants(
        9,
        8,
        13,
        0.15478515625
    );
    public static final SwerveModuleConstants kBackRightSwerve = new SwerveModuleConstants(
        4,
        3,
        16,
        -0.399169921875 + 0.5
    );
    public static final SwerveModuleConstants kBackLeftSwerve = new SwerveModuleConstants(
        2,
        1,
        15,
        0.56298828125
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

    public static final int kIntakeRaiserId = 5;
    public static final double kIntakeHomeVoltage = -4;
    public static final double kIntakeRaiserKP = 1;
    public static final double kIntakeRaiserKI = 0;
    public static final double kIntakeRaiserKD = 0;

    public static final double kIntakeUpSetpoint = 0;
    public static final double kIntakeDownSetpoint = 70.47813415527344;
    
    public static final int kIntakeSpinnerId = 10;
    public static final double kIntakeSpinnerVoltage = -6;

    public static final int kShooterId = 11;
    public static final double kShooterVoltage = 10;
    public static final double kShooterTargetRPM = 3000;
    public static final double kShooterKP = 0.00025669;
    public static final double kShooterKI = 0;
    public static final double kShooterKD = 0;

    public static final int kFeederId = 12;
    public static final double kFeederVoltage = -7;

}
