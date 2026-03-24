// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FeatureFlags;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intakeRaiser;

  private double m_calibrationOffset = 0;
  private boolean m_isCalibrated = false;

  private final PIDController m_intakeRaiserPID = new PIDController(
      Constants.kIntakeRaiserKP,
      Constants.kIntakeRaiserKI,
      Constants.kIntakeRaiserKD
  );

  private final DigitalInput m_homeTrigger = new DigitalInput(0);

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_intakeRaiser = new SparkMax(Constants.kIntakeRaiserId, MotorType.kBrushless);
    m_intakeRaiser.configure(
      Constants.kNeoNominalConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_intakeRaiserPID.calculate(getPosition(), getPosition());
  }

  public boolean isCalibrated() {
    return m_isCalibrated;
  }
  
  public void configure(SparkBaseConfig config) {
    m_intakeRaiser.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Assembly Position", getPosition());

    if (m_homeTrigger.get()) {
      m_calibrationOffset = m_intakeRaiser.getEncoder().getPosition();
      m_isCalibrated = true;
    }

    if (FeatureFlags.kEnableIntakeAssemblyMotion) {
      if (m_isCalibrated)
        m_intakeRaiser.setVoltage(
          m_intakeRaiserPID.calculate(getPosition())
        );
    }
  }

  public void setVoltage(double volts) {
    m_intakeRaiser.setVoltage(volts);
  }

  public void moveUp() {
    if (FeatureFlags.kEnableIntakeAssemblyMotion)
      m_intakeRaiser.setVoltage(
        m_intakeRaiserPID.calculate(getPosition(), Constants.kIntakeUpSetpoint)
      );
  }

  public void moveDown() {
    if (FeatureFlags.kEnableIntakeAssemblyMotion)
      m_intakeRaiser.setVoltage(
        m_intakeRaiserPID.calculate(getPosition(), Constants.kIntakeDownSetpoint)
      );
  }

  public double getPosition() {
    return m_intakeRaiser.getEncoder().getPosition() - m_calibrationOffset;
  }
}
