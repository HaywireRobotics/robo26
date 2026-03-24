// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.SwerveModule;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_shooter;
  private boolean isEStopped = false;

  private final PIDController m_pid = new PIDController(
    Constants.kShooterKP,
    Constants.kShooterKI,
    Constants.kShooterKD
  );
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
    0.24896,
    0.0021233,
    0.00031836
  );

  private boolean disablePIDControl = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_shooter = new SparkMax(Constants.kShooterId, MotorType.kBrushless);
    m_shooter.configure(
      Constants.kNeoNoBreakConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    SmartDashboard.putNumber("Shooter RPM", Constants.kShooterTargetRPM);
    SmartDashboard.putData("Shooter PID", m_pid);
    m_pid.calculate(getRPM(), 0);
    disablePIDControl = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!disablePIDControl)
      m_shooter.setVoltage( m_pid.calculate(getRPM()) + m_feedforward.calculate(m_pid.getSetpoint()) );
    SmartDashboard.putNumber("Shooter Current RPM", getRPM());
  }

  public void resetEStop() {
    isEStopped = false;
    m_shooter.configure(
      Constants.kNeoNoBreakConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  public void spinUp() {
    if (isEStopped) {
      return;
    }
    disablePIDControl = false;
    double RPM = getRPM();
    m_shooter.setVoltage( m_pid.calculate(RPM, SmartDashboard.getNumber("Shooter RPM", Constants.kShooterTargetRPM)) );
  }

  public double getRPM() {
    return m_shooter.getEncoder().getVelocity();
  }

  public void spinDown() {
    disablePIDControl = true;
    m_shooter.setVoltage( 0 );
  }

  public void eStop() {
    m_shooter.configure(
      Constants.kNeoNominalConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    disablePIDControl = true;
    m_shooter.setVoltage(-1);
    m_pid.calculate(getRPM(), 0);
    isEStopped = true;
  }

  public void disablePID() {
    disablePIDControl = true;
  }

  public void enablePID() {
    disablePIDControl = false;
  }

  public boolean isPIDDisabled() {
    return disablePIDControl;
  }

  // System Identification
  
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Units.Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Units.Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = Units.MetersPerSecond.mutable(0);

  public void sysIdVoltageDrive(Voltage voltage){
    m_appliedVoltage.mut_replace(voltage);
    m_shooter.setVoltage(voltage.in(Units.Volts));
  }

  public SysIdRoutineLog driveLogs(SysIdRoutineLog logs) {
    logs.motor("Shooter")
      .linearVelocity(m_velocity      .mut_replace( m_shooter.getEncoder().getVelocity() ,Units.MetersPerSecond))
      .voltage(       m_appliedVoltage)
      .linearPosition(m_distance      .mut_replace( m_shooter.getEncoder().getPosition() ,Units.Meters         ));
    
    System.out.printf("Velocity: %f rpm Voltage: %f volts Position: %f rotations\n", m_shooter.getEncoder().getVelocity(), m_appliedVoltage.in(Units.Volts), m_shooter.getEncoder().getPosition());
  
    return logs;
  }
}
