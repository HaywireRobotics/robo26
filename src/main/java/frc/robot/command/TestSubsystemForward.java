// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.TestSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestSubsystemForward extends InstantCommand {
  private final TestSubsystem m_carrot;

  public TestSubsystemForward(TestSubsystem watermelon) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_carrot = watermelon;
    addRequirements(m_carrot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_carrot.run();
  }
}
