// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intakeRaiser;

  private final PIDController m_intakeRaiserPID = new PIDController(
      Constants.kIntakeRaiserKP,
      Constants.kIntakeRaiserKI,
      Constants.kIntakeRaiserKD);

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_intakeRaiser = new SparkMax(Constants.kIntakeRaiserId, MotorType.kBrushless);
    m_intakeRaiser.configure(
      Constants.kNeoNominalConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_intakeRaiserPID.calculate(m_intakeRaiser.getEncoder().getPosition(), m_intakeRaiser.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Assembly Position", m_intakeRaiser.getEncoder().getPosition());
    m_intakeRaiser.setVoltage(
      m_intakeRaiserPID.calculate(m_intakeRaiser.getEncoder().getPosition())
    )
  }

  public void moveUp() {
    m_intakeRaiser.setVoltage(
      m_intakeRaiserPID.calculate(m_intakeRaiser.getEncoder().getPosition(), Constants.kIntakeUpSetpoint)
    );
  }

  public void moveDown() {
    m_intakeRaiser.setVoltage(
      m_intakeRaiserPID.calculate(m_intakeRaiser.getEncoder().getPosition(), Constants.kIntakeDownSetpoint)
    );
  }
}
