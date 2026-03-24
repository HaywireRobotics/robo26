// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_testCommand;
  private Command m_teleopCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer(this);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_robotContainer.updateOdometry();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.onDisable();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.whileDisabled();
  }

  @Override
  public void disabledExit() {
    m_robotContainer.onExitDisable();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_teleopCommand = m_robotContainer.getTeleopCommand();

    CommandScheduler.getInstance().schedule(m_teleopCommand);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    if (m_teleopCommand != null) {
      m_teleopCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    m_testCommand = m_robotContainer.getTestCommand();

    if (m_testCommand != null) {
      CommandScheduler.getInstance().schedule(m_testCommand);
    }
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
