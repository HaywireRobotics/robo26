// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ChooChooTrain;
import frc.robot.subsystem.LEDSuperSystem;
import frc.robot.wrappers.Controller;
import frc.robot.wrappers.HID_Keyturn;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ThisIsAKeyStart extends Command {
  private final HID_Keyturn m_key = new HID_Keyturn(1);
  private final ChooChooTrain m_DriveTrain;
  private int keyStartTime = 0;
  private Controller m_controller;

  /** Creates a new ThisIsAKeyStart. */
  public ThisIsAKeyStart(ChooChooTrain driveTrain, Controller controller, LEDSuperSystem headlights) {
    m_DriveTrain = driveTrain;
    m_controller = controller;
    m_LEDs = headlights;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);

  }
  private final LEDSuperSystem m_LEDs;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    keyStartTime = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Keystart", m_key.GetKeyState());
    SmartDashboard.putNumber("keyStartTime", keyStartTime);
    if (m_key.GetKeyState() && keyStartTime < 500) {
      keyStartTime ++ ;
    }
    else if (keyStartTime > 0){
      keyStartTime -= 50;

    }
    if (keyStartTime < 0){
      keyStartTime = -0;
    }
    if (keyStartTime > 0){
      m_LEDs.getAddressableLEDBuffer().setRGB(20, 255, 255, 255);
      m_LEDs.getAddressableLEDBuffer().setRGB(40, 255, 255, 255);
      m_LEDs.updateLEDBuffer();
      
    }
    double rotations = 0;
    if (m_controller.x().getAsBoolean()) {
      rotations += 5;
    }
    if (m_controller.b().getAsBoolean()) {
      rotations -= 5;
    }
    m_DriveTrain.drive(0, -(double)keyStartTime/250.0, rotations, false);
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
