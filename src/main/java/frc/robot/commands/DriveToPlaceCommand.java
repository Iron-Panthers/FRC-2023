// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivebaseSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPlaceCommand extends InstantCommand {
  private final DrivebaseSubsystem drivebaseSubsystem;
  private final PathPoint pathPoint;

  public DriveToPlaceCommand(DrivebaseSubsystem drivebaseSubsystem, PathPoint pathPoint) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.pathPoint = pathPoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(3, 3),
            new PathPoint(
                drivebaseSubsystem.getPose().getTranslation(),
                drivebaseSubsystem.getGyroscopeRotation()),
            pathPoint);
    this.andThen(new FollowTrajectoryCommand(trajectory, drivebaseSubsystem));
  }
}
