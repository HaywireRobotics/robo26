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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.command.Circle;
import frc.robot.command.DefaultTrainDriveCommand;
import frc.robot.command.IntakeDownCommand;
import frc.robot.command.IntakeUpCommand;
import frc.robot.command.LEDCompassCommand;
import frc.robot.command.RunFeederCommand;
import frc.robot.command.ShooterEStopCommand;
import frc.robot.command.ShooterSpinCommand;
import frc.robot.command.SpinIntakeCommand;
import frc.robot.command.ThisIsAKeyStart;
import frc.robot.command.TuneSwerveAutonomousCommand;
import frc.robot.subsystem.ChooChooTrain;
import frc.robot.subsystem.FeederSubsystem;
import frc.robot.subsystem.IntakeSpinnerSubsystem;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.LEDSuperSystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.wrappers.Camera;
import frc.robot.wrappers.Controller;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  public final ChooChooTrain drivetrain;
  private final LEDSuperSystem m_leds;

  private final IntakeSpinnerSubsystem m_intakeSpinner;
  private final IntakeSubsystem m_intake;

  private final FeederSubsystem m_feeder;

  private final ShooterSubsystem m_shooter;

  private final DefaultTrainDriveCommand m_defaultDriveCommand;
  private final LEDCompassCommand m_defaultLEDCommand;

  private final Controller m_driverController;
  private final Controller m_manipulatorController;

  private final Camera m_camera = new Camera("Camera_Module_v1", new Transform3d(
    new Translation3d(),
    new Rotation3d()
  ));

  public RobotContainer(Robot robot) {
    m_driverController      = new Controller(0);
    m_manipulatorController = new Controller(1);
    drivetrain              = new ChooChooTrain(robot);
    m_leds                  = new LEDSuperSystem();
    m_intake                = new IntakeSubsystem();
    m_intakeSpinner         = new IntakeSpinnerSubsystem();
    m_shooter               = new ShooterSubsystem();
    m_feeder                = new FeederSubsystem();

    m_defaultDriveCommand = new DefaultTrainDriveCommand(drivetrain, m_driverController);
    m_defaultLEDCommand = new LEDCompassCommand(drivetrain, m_leds);

    drivetrain.setDefaultCommand(m_defaultDriveCommand);
    m_leds.setDefaultCommand(m_defaultLEDCommand);

    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);
  }

  private void configureBindings() {
    m_driverController.a().onTrue( new Circle(drivetrain) );

    // m_manipulatorController.getByName(ButtonMappings.manipulatorIntakeDown).onTrue( new IntakeDownCommand(m_intake) );
    // m_manipulatorController.getByName(ButtonMappings.manipulatorIntakeUp).onTrue( new IntakeUpCommand(m_intake) );
    m_manipulatorController.getByName(ButtonMappings.manipulatorIntakeSpin).whileTrue( new SpinIntakeCommand(m_intakeSpinner) );

    m_manipulatorController.getByName(ButtonMappings.manipulatorShooterSpin).whileTrue(   new ShooterSpinCommand(m_shooter)  );
    m_manipulatorController.getByName(ButtonMappings.manipulatorShooterEStop).whileTrue(  new ShooterEStopCommand(m_shooter) );
    m_manipulatorController.getByName(ButtonMappings.manipulatorFeederForward).whileTrue( new RunFeederCommand(m_feeder)     );
  }

  public void onDisable() {
    m_shooter.resetEStop();
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
    // return new ThisIsAKeyStart(drivetrain, m_driverController, m_leds);
    return new TuneSwerveAutonomousCommand(drivetrain);
  }
}
