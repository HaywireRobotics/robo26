// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.command.Circle;
import frc.robot.command.DefaultTrainDriveCommand;
import frc.robot.command.LEDCompassCommand;
import frc.robot.command.ThisIsAKeyStart;
import frc.robot.command.TuneSwerveAutonomousCommand;
import frc.robot.subsystem.ChooChooTrain;
import frc.robot.subsystem.LEDSuperSystem;
import frc.robot.wrappers.Camera;
import frc.robot.wrappers.Controller;
import frc.robot.wrappers.CustomRender;
import frc.robot.wrappers.WasmWrapper;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  public final ChooChooTrain drivetrain;
  private final LEDSuperSystem m_leds;

  private final DefaultTrainDriveCommand m_defaultDriveCommand;
  private final LEDCompassCommand m_defaultLEDCommand;

  private CustomRender renderer;
  private final Controller m_driverController;

  private final Camera m_camera = new Camera("Camera_Module_v1", new Transform3d(
    new Translation3d(),
    new Rotation3d()
  ));

  public RobotContainer(Robot robot) {
    m_driverController = new Controller(0);
    drivetrain         = new ChooChooTrain(robot);
    m_leds             = new LEDSuperSystem();

    renderer = new CustomRender("Renderer", 100, 100);

    m_defaultDriveCommand = new DefaultTrainDriveCommand(drivetrain, m_driverController);
    m_defaultLEDCommand = new LEDCompassCommand(drivetrain, m_leds);

    drivetrain.setDefaultCommand(m_defaultDriveCommand);
    m_leds.setDefaultCommand(m_defaultLEDCommand);

    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);
  }


  private void configureBindings() {
    m_driverController.a().onTrue( new Circle(drivetrain));
    m_driverController.a().onTrue( 
      new FunctionalCommand(() -> {}, () -> {}, (boolean none) -> {}, () -> {return false;})
    );
    m_driverController.b().whileTrue( WasmWrapper.getCommand("MyWasmCommand") );
  }

  public void updateOdometry() {
    drivetrain.updateOdometry();
    // Optional<Pose2d> estimated_pose = m_camera.estimatePose(m_drivetrain.getPose2D());

    // if (estimated_pose.isPresent()) {
    //   m_drivetrain.setOdometry(estimated_pose.get());
    // }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command getTestCommand() {
    return new ThisIsAKeyStart(drivetrain, m_driverController, m_leds);
  }
}
