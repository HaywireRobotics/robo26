// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.wrappers.SwerveModule;

public class ChooChooTrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.kDriveTrainLength/2, Constants.kDriveTrainWidth/2);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.kDriveTrainLength/2, -Constants.kDriveTrainWidth/2);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.kDriveTrainLength/2, Constants.kDriveTrainWidth/2);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.kDriveTrainLength/2, -Constants.kDriveTrainWidth/2);


  private final SwerveModule m_frontRight = new SwerveModule(
    Constants.kFrontRightSwerve.driveMotor,
    Constants.kFrontRightSwerve.turnMotor,
    Constants.kFrontRightSwerve.encoder,
    Constants.kFrontRightSwerve.offset);
  private final SwerveModule m_frontLeft = new SwerveModule(
    Constants.kFrontLeftSwerve.driveMotor,
    Constants.kFrontLeftSwerve.turnMotor,
    Constants.kFrontLeftSwerve.encoder,
    Constants.kFrontLeftSwerve.offset);
  private final SwerveModule m_backRight = new SwerveModule(
    Constants.kBackRightSwerve.driveMotor,
    Constants.kBackRightSwerve.turnMotor,
    Constants.kBackRightSwerve.encoder,
    Constants.kBackRightSwerve.offset);
  private final SwerveModule m_backLeft = new SwerveModule(
    Constants.kBackLeftSwerve.driveMotor,
    Constants.kBackLeftSwerve.turnMotor,
    Constants.kBackLeftSwerve.encoder,
    Constants.kBackLeftSwerve.offset);
  private final SwerveModule[] m_swerveModules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};

  private final String[] swerveModuleNames = {"front left", "back left", "front right", "back right"};
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Units.Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Units.Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = Units.MetersPerSecond.mutable(0);
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  private Rotation2d m_calibratedOffset = new Rotation2d();

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation,
          m_frontRightLocation,
          m_backLeftLocation,
          m_backRightLocation);

  private final SwerveDriveOdometry m_odometry;
  private Pose2d m_fieldPose = new Pose2d();

  private final Robot m_robot;

  public ChooChooTrain(Robot robot) {
    this.resetGyro();
    m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      getRotationAroundUpAxisInRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      }
    );
    m_robot = robot;

    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception error) {
      System.out.println("Could not load config from file!");
      throw new Error("Could not load RobotConfig from file");
    }

    AutoBuilder.configure(
      this::getPose2D,
      this::setOdometry,
      this::getChassisSpeeds,
      this::reversedDrive,
      new PPHolonomicDriveController(
        new PIDConstants(4, 0, 0),
        new PIDConstants(4, 0.1, 0)
      ),
      robotConfig,
      () -> { // Flip Auto reporter
        return false;
      },
      this
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    drive(fieldRelative ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rot, getRotationAroundUpAxisInRotation2d())
      :
      new ChassisSpeeds(ySpeed, xSpeed, rot)
    );
  }

  public void drive(ChassisSpeeds speed) {
    SmartDashboard.putNumber("X Speed", speed.vxMetersPerSecond);
    SmartDashboard.putNumber("Y Speed", speed.vyMetersPerSecond);
    SmartDashboard.putNumber("Rotations", speed.omegaRadiansPerSecond);
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                speed,
                m_robot.getPeriod()));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void reversedDrive(ChassisSpeeds speed) {
    speed.omegaRadiansPerSecond = -speed.omegaRadiansPerSecond;
    drive(speed);
  }

  public void sysIdVoltageDrive(Voltage voltage){
    setAllToState(new SwerveModuleState(voltage.in(Units.Volts), new Rotation2d(0)));
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_fieldPose = m_odometry.update(
      getRotationAroundUpAxisInRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      });
  }

  public void stopAll() {
    m_frontLeft.driveMotorsAtVoltage(0, 0);
    m_frontRight.driveMotorsAtVoltage(0, 0);
    m_backLeft.driveMotorsAtVoltage(0, 0);
    m_backRight.driveMotorsAtVoltage(0, 0);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
      m_frontLeft.getPositionState(),
      m_frontRight.getPositionState(),
      m_backLeft.getPositionState(),
      m_backRight.getPositionState()
    );
  }

  public void setOdometry(Pose2d newPose) {
    m_odometry.resetPose(newPose);
    m_fieldPose = newPose;
  }
 
  public Pose2d getFieldPose(){
    return m_fieldPose;
  }

  public void setAllToState(SwerveModuleState state){
    m_frontLeft.setDesiredState(state);
    m_frontRight.setDesiredState(state);
    m_backLeft.setDesiredState(state);
    m_backRight.setDesiredState(state);
  }

  public void periodic(){
    for(SwerveModule module : m_swerveModules){
      module.periodic();
    }
  }

  public double[] getSwerveDriveLocations() {
    return new double[] {
      m_frontLeft.getPosition().distanceMeters,
      m_frontRight.getPosition().distanceMeters,
      m_backLeft.getPosition().distanceMeters,
      m_backRight.getPosition().distanceMeters
    };
  }

  public double getRotationAroundUpAxis() {
    return m_gyro.getYaw();
  }

  public Rotation2d getRotationAroundUpAxisInRotation2d() {
    return m_gyro.getRotation2d().minus(m_calibratedOffset);
  }

  public SysIdRoutineLog driveLogs(SysIdRoutineLog logs){
    for(int i = 0; i < m_swerveModules.length;i++){
      SwerveModule module = m_swerveModules[i];
      logs.motor("motor" + swerveModuleNames[i])
        .linearVelocity(m_velocity.mut_replace(module.getDriveVelocity(),Units.MetersPerSecond))
        .voltage(m_appliedVoltage.mut_replace(module.getDriveVoltage(),Units.Volts))
        .linearPosition(m_distance.mut_replace(module.getDrivePosition(),Units.Meters));
    }
   return logs;
  }

  public Pose2d getPose2D() {
    return m_fieldPose;
  }

  public void resetGyro() {
    // m_gyro.reset();
    m_calibratedOffset = m_gyro.getRotation2d();
  }

  public Command getPathByName(String name) {
    try{
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public void configure(SparkBaseConfig config) {
    m_frontLeft.configure(config);
    m_frontRight.configure(config);
    m_backLeft.configure(config);
    m_backRight.configure(config);
  }
}
