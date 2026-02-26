// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Statics;
import frc.robot.subsystem.ChooChooTrain;
import frc.robot.subsystem.LEDSuperSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDCompassCommand extends Command { 
  private final ChooChooTrain m_driveTrain;
  private final LEDSuperSystem m_LEDs;
  /** Creates a new LEDCompassCommand. */
  public LEDCompassCommand(ChooChooTrain driveTrain, LEDSuperSystem LEDs) {
    m_driveTrain = driveTrain;
    m_LEDs = LEDs;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private int previous_index = 0;
  private int triggers = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d compassangle = m_driveTrain.getRotationAroundUpAxisInRotation2d();
    SmartDashboard.putNumber("CompassRadians", compassangle.getRadians());
    int index = Statics.angleToPointOnLEDStrip(
      compassangle.getRadians(),
      18,
      95
    );
    SmartDashboard.putNumber("CompassIndex", index);
    SmartDashboard.putNumber("CompassPrevIndex", previous_index);
    SmartDashboard.putNumber("CompassTriggers", triggers);
    if (index != previous_index) {
      triggers++;
      m_LEDs.getAddressableLEDBuffer().setRGB(index, 255, 255, 255);
      m_LEDs.getAddressableLEDBuffer().setRGB(previous_index, 0, 0, 0);
      previous_index = index;
      m_LEDs.updateLEDBuffer();
    }
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
