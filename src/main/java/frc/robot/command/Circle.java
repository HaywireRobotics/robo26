// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ChooChooTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Circle extends Command {
  private final ChooChooTrain spin ;
  private double spin_progress = 0 ;

  private double scalar = 0.1;
  
  /** Creates a new Circle. */
  public Circle(ChooChooTrain spinner) {
    spin = spinner;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spin);
    
    SmartDashboard.putNumber("CircleSpeed", 0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spin_progress = 0;

    scalar = SmartDashboard.getNumber("CircleSpeed", 0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spin_progress += 0.02;
    spin.drive(Math.sin(spin_progress) * scalar, Math.cos(spin_progress) * scalar, 0,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spin.drive( 0,0,0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return spin_progress >= Math.PI * 2;
  
  }
}
