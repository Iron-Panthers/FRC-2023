// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivebaseSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPlaceCommand extends InstantCommand {
  private final DrivebaseSubsystem drivebaseSubsystem;
  private final Translation2d translation2d;

  public DriveToPlaceCommand(DrivebaseSubsystem drivebaseSubsystem, Translation2d translation2d) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.translation2d = translation2d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(3, 3),
            // start point
            new PathPoint(
                drivebaseSubsystem.getPose().getTranslation(),
                Rotation2d.fromDegrees(0),
                drivebaseSubsystem.getGyroscopeRotation()),
            new PathPoint(
                translation2d,
                Rotation2d.fromDegrees(0),
                drivebaseSubsystem.getGyroscopeRotation()));
    CommandScheduler.getInstance()
        .schedule((new FollowTrajectoryCommand(trajectory, drivebaseSubsystem)));
  }
}
