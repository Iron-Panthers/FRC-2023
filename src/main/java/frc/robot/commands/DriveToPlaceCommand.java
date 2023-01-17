// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.AdvancedSwerveTrajectoryFollower;
import frc.util.Util;

public class DriveToPlaceCommand extends CommandBase {

  private final DrivebaseSubsystem drivebaseSubsystem;
  private final Pose2d finalPose;

  private final AdvancedSwerveTrajectoryFollower follower;

  private final double observationDistance;
  private final double observationTime;

  PathPlannerTrajectory trajectory;
  double generationTime;
  double adjustCount;

  /** Creates a new DriveToPlaceCommand. */
  public DriveToPlaceCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      Pose2d finalPose,
      double observationDistance,
      double observationTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.finalPose = finalPose;
    this.observationDistance = observationDistance;
    this.observationTime = observationTime;

    follower = drivebaseSubsystem.getFollower();

    addRequirements(drivebaseSubsystem);
  }

  private Rotation2d straightLineAngle(Translation2d start, Translation2d end) {
    double x1 = start.getX();
    double y1 = start.getY();
    double x2 = end.getX();
    double y2 = end.getY();

    double angle = Math.atan2(y2 - y1, x2 - x1);
    return Rotation2d.fromRadians(angle);
  }

  private PathPlannerTrajectory createObservationTrajectory() {
    var currentPose = drivebaseSubsystem.getPose();
    var initialPoint =
        new PathPoint(
            currentPose.getTranslation(),
            straightLineAngle(currentPose.getTranslation(), finalPose.getTranslation()),
            // holonomic rotation should start at our current rotation
            currentPose.getRotation());

    var observationPoint =
        new PathPoint(
            // drive until we are .2 meter away from the final position
            finalPose
                .getTranslation()
                .minus(
                    new Translation2d(-observationDistance, 0)
                        .rotateBy(
                            straightLineAngle(
                                finalPose.getTranslation(), currentPose.getTranslation()))),
            // drive in a straight line to the final position
            straightLineAngle(currentPose.getTranslation(), finalPose.getTranslation()),
            // holonomic rotation should be the same as the final rotation to ensure tag visibility
            finalPose.getRotation());

    return PathPlanner.generatePath(
        new PathConstraints(5, 1.5),
        initialPoint,
        // stop near the goal to read apriltags
        observationPoint);
  }

  private PathPlannerTrajectory createAdjustTrajectory() {
    var currentPose = drivebaseSubsystem.getPose();
    var initialPoint =
        new PathPoint(
            currentPose.getTranslation(),
            straightLineAngle(currentPose.getTranslation(), finalPose.getTranslation()),
            // holonomic rotation should start at our current rotation
            currentPose.getRotation());

    var finalPoint =
        new PathPoint(
            // drive into the final position
            finalPose.getTranslation(),
            straightLineAngle(finalPose.getTranslation(), currentPose.getTranslation()),
            finalPose.getRotation());

    return PathPlanner.generatePath(new PathConstraints(1, .5), initialPoint, finalPoint);
  }

  private double distanceBetween(Pose2d p1, Pose2d p2) {
    return p1.getTranslation().getDistance(p2.getTranslation());
  }

  private boolean finishedPath() {
    return Timer.getFPGATimestamp() - generationTime
        > (trajectory.getTotalTimeSeconds() + observationTime);
  }

  private boolean poseSatisfied() {
    return AdvancedSwerveTrajectoryFollower.poseWithinErrorMarginOfTrajectoryFinalGoal(
        finalPose, trajectory);
  }

  private PathPlannerTrajectory createNextTrajectory() {
    generationTime = Timer.getFPGATimestamp();
    return distanceBetween(drivebaseSubsystem.getPose(), finalPose) < observationDistance
        ? createAdjustTrajectory()
        : createObservationTrajectory();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    adjustCount = 0;
    generationTime = Timer.getFPGATimestamp();
    trajectory = createNextTrajectory();
    follower.follow(trajectory);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // print the distance to the final pose
    var fP =
        ((PathPlannerState)
            trajectory
                // sample the final position using the time greater than total time behavior
                .sample(trajectory.getTotalTimeSeconds() + 1));
    System.out.println(
        String.format(
            "    xy err: %8f theta err: %8f",
            drivebaseSubsystem
                .getPose()
                .getTranslation()
                .getDistance(fP.poseMeters.getTranslation()),
            Util.relativeAngularDifference(
                drivebaseSubsystem.getPose().getRotation(), fP.holonomicRotation)));

    if (finishedPath()) {
      if (poseSatisfied()) {
        this.cancel();
      }
      System.out.println("creating new trajectory");
      trajectory = createNextTrajectory();
      follower.follow(trajectory);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    follower.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedPath() && poseSatisfied();
  }
}
