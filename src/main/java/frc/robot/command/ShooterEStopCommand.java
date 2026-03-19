// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterEStopCommand extends InstantCommand {
  private final ShooterSubsystem m_shooter;

  public ShooterEStopCommand(ShooterSubsystem shooter) {
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.eStop();
    System.err.println("/------------/ ESTOP TRIGGERED /------------/");
    System.err.println("The EStop for the shooter has been triggered ");
    System.err.println("and the shooter should stop under any condit-");
    System.err.println("ion. This may damage the shooter motor, but  ");
    System.err.println("keep everything else safe.");
  }
}
