// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Statics;
import frc.robot.subsystem.ChooChooTrain;
import frc.robot.wrappers.Controller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultTrainDriveCommand extends Command {
  private final ChooChooTrain m_drive;
  private final Controller m_controller;

  private boolean m_fieldRelative = false;

  //the buffer not only affects the "deadzone", 
  // but also prohibits small angles near the x/y axis
  private static final double JOYSTICK_DEADBAND = 0.1;
  public double teleopSpeedMultiplier = 1.6;

  /** Creates a new DefaultTrainDriveCommand. */
  public DefaultTrainDriveCommand(ChooChooTrain drive, Controller controller) {
    m_drive = drive;
    m_controller = controller;

    addRequirements(m_drive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightX = m_controller.getRightX();
    double leftX = m_controller.getLeftX();
    double leftY = m_controller.getLeftY();

    leftX = Statics.applyDeadband(leftX, JOYSTICK_DEADBAND);
    leftY = Statics.applyDeadband(leftY, JOYSTICK_DEADBAND);
    rightX = Statics.applyDeadband(rightX, JOYSTICK_DEADBAND);

    double slowdownValue = (Constants.kSlowModeDivider - m_controller.getLeftTriggerAxis()) / Constants.kSlowModeDivider;
    
    leftX = leftX * slowdownValue;
    leftY = leftY * slowdownValue;
    rightX = rightX * slowdownValue;


    m_drive.drive(Constants.kNavigationMultiplier*leftX, Constants.kNavigationMultiplier*leftY, -Constants.kRotationMultiplier*rightX,m_fieldRelative);
    
    /* Odometry */
    // m_subsystem.updateOdometry();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
