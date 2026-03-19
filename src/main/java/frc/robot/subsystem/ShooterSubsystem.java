// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_shooter;
  private boolean isEStopped = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_shooter = new SparkMax(Constants.kShooterId, MotorType.kBrushless);
    m_shooter.configure(
      Constants.kNeoNoBreakConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    SmartDashboard.putNumber("Shooter Voltage", Constants.kShooterVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    m_shooter.setVoltage(SmartDashboard.getNumber("Shooter Voltage", Constants.kShooterVoltage));
  }

  public void spinDown() {
    m_shooter.setVoltage(0);
  }

  public void eStop() {
    m_shooter.configure(
      Constants.kNeoNominalConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_shooter.setVoltage(-1);
    isEStopped = true;
  }
}
