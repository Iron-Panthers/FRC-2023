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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.AdvancedSwerveTrajectoryFollower;

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

  private Rotation2d computeStartingHeading(Translation2d start, Translation2d end) {
    double x1 = start.getX();
    double y1 = start.getY();
    double x2 = end.getX();
    double y2 = end.getY();

    double angle = Math.atan2(y2 - y1, x2 - x1);
    return Rotation2d.fromRadians(angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectory =
        PathPlanner.generatePath(
            new PathConstraints(3, .5),
            new PathPoint(
                drivebaseSubsystem.getPose().getTranslation(),
                computeStartingHeading(
                    drivebaseSubsystem.getPose().getTranslation(), finalPose.getTranslation()),
                // holonomic rotation should start at our current rotation
                drivebaseSubsystem.getGyroscopeRotation()),
            new PathPoint(
                // drive into the final position from the back
                finalPose.getTranslation(),
                computeStartingHeading(
                        drivebaseSubsystem.getPose().getTranslation(), finalPose.getTranslation())
                    .plus(Rotation2d.fromDegrees(180)),
                finalPose.getRotation()));

    drivebaseSubsystem.getFollower().setRunUntilAccurate(true);
    drivebaseSubsystem.getFollower().follow(trajectory);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // print the distance to the final pose
    System.out.println(
        "Distance to final pose: "
            + drivebaseSubsystem
                .getPose()
                .getTranslation()
                .getDistance(finalPose.getTranslation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.getFollower().cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return AdvancedSwerveTrajectoryFollower.poseWithinErrorMarginOfTrajectoryFinalGoal(
        drivebaseSubsystem.getPose(), trajectory);
  }
}
