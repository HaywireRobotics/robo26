// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.command.DefaultTrainDriveCommand;
import frc.robot.command.TuneSwerveAutonomousCommand;
import frc.robot.subsystem.ChooChooTrain;
import frc.robot.wrappers.Controller;

public class RobotContainer {
  private final ChooChooTrain m_drivetrain;

  private final DefaultTrainDriveCommand m_defaultDriveCommand;

  private final Controller m_driverController;

  public RobotContainer(Robot robot) {
    m_driverController = new Controller(0);
    m_drivetrain = new ChooChooTrain(robot);

    m_defaultDriveCommand = new DefaultTrainDriveCommand(m_drivetrain, m_driverController);

    m_drivetrain.setDefaultCommand(m_defaultDriveCommand);

    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return new TuneSwerveAutonomousCommand(m_drivetrain);
  }
}
