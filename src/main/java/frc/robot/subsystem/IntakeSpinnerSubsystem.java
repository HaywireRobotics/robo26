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

public class IntakeSpinnerSubsystem extends SubsystemBase {
  private final SparkMax m_intakeSpinner;
  /** Creates a new IntakeSpinnerSubsystem. */
  public IntakeSpinnerSubsystem() {
    m_intakeSpinner = new SparkMax(Constants.kIntakeSpinnerId, MotorType.kBrushless);
    m_intakeSpinner.configure(Constants.kNeoNominalConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    SmartDashboard.putNumber("Intake Voltage", Constants.kIntakeSpinnerVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void runSpinner() {
    m_intakeSpinner.setVoltage(SmartDashboard.getNumber("Intake Voltage", Constants.kIntakeSpinnerVoltage));
  }

  public void stopSpinner() {
    m_intakeSpinner.setVoltage(0);
  }
}
