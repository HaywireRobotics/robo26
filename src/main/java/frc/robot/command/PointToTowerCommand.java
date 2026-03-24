// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ChooChooTrain;
import frc.robot.wrappers.TagPose2d;

public class PointToTowerCommand extends Command {
  private final ChooChooTrain m_drivetrain;

  private final HolonomicDriveController m_controller;
  private Trajectory m_trajectory;
  private final Timer m_timer;

  private Pose2d target;

  private static final Pose2d blueTower = new Pose2d(
    new Translation2d(4.03 + (1.19 / 2), 8.07 / 2),
    new Rotation2d()
  );
  private static final Pose2d redTower = new Pose2d(
    new Translation2d(16.54 - (4.03 + (1.19 / 2)), 8.07 / 2),
    new Rotation2d()
  );

  private static final double field_midpoint = 16.54 / 2;

  /** Creates a new PointToTowerCommand. */
  public PointToTowerCommand(ChooChooTrain drivetrain) {
    m_drivetrain = drivetrain;

    final ProfiledPIDController headingController = new ProfiledPIDController(5, 0.1, 0, new TrapezoidProfile.Constraints(6.28, 6.28));
    m_controller = new HolonomicDriveController(
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      headingController
    );
    m_timer = new Timer();

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = m_drivetrain.getPose2D();

    boolean isRedSide = pose.getX() > field_midpoint;

    m_timer.reset();
    m_timer.start();
    final ArrayList<Translation2d> waypoints = new ArrayList<>(2);

    m_trajectory = TrajectoryGenerator.generateTrajectory(
      m_drivetrain.getPose2D(),
      waypoints,
      new TagPose2d(pose)
        .facing(isRedSide ? redTower : blueTower).toPose(),
      new TrajectoryConfig(0.5, 1));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State reference = m_trajectory.sample(m_timer.get());

    Rotation2d targetRotation = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters.getRotation();
    ChassisSpeeds movement = m_controller.calculate(m_drivetrain.getPose2D(), reference, targetRotation);
    movement.omegaRadiansPerSecond = -movement.omegaRadiansPerSecond;

    m_drivetrain.drive(movement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
