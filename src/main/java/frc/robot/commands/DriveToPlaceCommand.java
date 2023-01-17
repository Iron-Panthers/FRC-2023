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
import frc.robot.Constants.PoseEstimator;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.AdvancedSwerveTrajectoryFollower;
import frc.util.Util;
import java.util.Optional;

public class DriveToPlaceCommand extends CommandBase {

  private final DrivebaseSubsystem drivebaseSubsystem;
  private final Pose2d finalPose;

  private final AdvancedSwerveTrajectoryFollower follower;

  private final double visionCalibrateOffset;

  private static final PathConstraints pathConstraints = new PathConstraints(5, 1.5);

  int stabilityCounter = 0;

  PathPlannerTrajectory trajectory;
  double generationTime;
  private final double repathDelaySeconds;
  double repathCount;

  /** Creates a new DriveToPlaceCommand. */
  public DriveToPlaceCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      Pose2d finalPose,
      double visionCalibrateOffset,
      double repathDelaySeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.finalPose = finalPose;
    this.visionCalibrateOffset = visionCalibrateOffset;
    this.repathDelaySeconds = repathDelaySeconds;

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

  private PathPlannerTrajectory createTrajectory() {
    var currentPose = drivebaseSubsystem.getPose();
    var initialPoint =
        new PathPoint(
            currentPose.getTranslation(),
            straightLineAngle(currentPose.getTranslation(), finalPose.getTranslation()),
            // holonomic rotation should start at our current rotation
            currentPose.getRotation());

    var intermediatePoint =
        new PathPoint(
            // drive until we are .2 meter away from the final position
            finalPose
                .getTranslation()
                .minus(
                    new Translation2d(-visionCalibrateOffset, 0)
                        .rotateBy(
                            straightLineAngle(
                                finalPose.getTranslation(), currentPose.getTranslation()))),
            // drive in a straight line to the final position
            straightLineAngle(currentPose.getTranslation(), finalPose.getTranslation()),
            // holonomic rotation should be the same as the final rotation to ensure tag visibility
            finalPose.getRotation());

    // var finalPoint =
    //     new PathPoint(
    //         // drive into the final position
    //         finalPose.getTranslation(),
    //         straightLineAngle(finalPose.getTranslation(), currentPose.getTranslation()),
    //         finalPose.getRotation());

    // if (currentPose.getTranslation().getDistance(finalPose.getTranslation())
    //     < visionCalibrateOffset) {
    //   return PathPlanner.generatePath(pathConstraints, initialPoint, finalPoint);
    // }

    return PathPlanner.generatePath(
        pathConstraints,
        initialPoint,
        // stop near the goal to read apriltags with zero velocity
        intermediatePoint);
  }

  private boolean shouldRepath() {
    return Timer.getFPGATimestamp() - generationTime
        > (trajectory.getTotalTimeSeconds() + repathDelaySeconds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stabilityCounter = 0;
    repathCount = 0;
    generationTime = Timer.getFPGATimestamp();
    trajectory = createTrajectory();

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
            "xy err: %8f theta err: %8f Vel: %8f Stability: %3d",
            drivebaseSubsystem
                .getPose()
                .getTranslation()
                .getDistance(fP.poseMeters.getTranslation()),
            Util.relativeAngularDifference(
                drivebaseSubsystem.getPose().getRotation(), fP.holonomicRotation),
            Optional.ofNullable(follower.getLastState())
                .map((s) -> s.velocityMetersPerSecond)
                .orElseGet(() -> -10000d),
            stabilityCounter));

    if (shouldRepath()) {
      if (repathCount > 0) {
        this.cancel();
      }
      generationTime = Timer.getFPGATimestamp();

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

      trajectory = PathPlanner.generatePath(new PathConstraints(1, .5), initialPoint, finalPoint);
      repathCount++;
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
    return stabilityCounter >= PoseEstimator.STABILITY_COUNT_THRESHOLD;
  }
}
