// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class DriveToPlaceCommand extends CommandBase {

  private final DrivebaseSubsystem drivebaseSubsystem;
  private final Pose2d finalPose;

  PathPlannerTrajectory trajectory;

  /** Creates a new DriveToPlaceCommand. */
  public DriveToPlaceCommand(DrivebaseSubsystem drivebaseSubsystem, Pose2d finalPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.finalPose = finalPose;

    addRequirements(drivebaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectory =
        PathPlanner.generatePath(
            new PathConstraints(3, 3),
            new PathPoint(
                drivebaseSubsystem.getPose().getTranslation(),
                Rotation2d.fromDegrees(0 /* use trig to draw line to goal */),
                // holonomic rotation should start at our current rotation
                drivebaseSubsystem.getGyroscopeRotation()),
            new PathPoint(
                finalPose.getTranslation(), Rotation2d.fromDegrees(0), finalPose.getRotation()));

    drivebaseSubsystem.getFollower().follow(trajectory);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.getFollower().cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
