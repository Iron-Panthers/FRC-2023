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
import java.util.Optional;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class DriveToPlaceCommand extends CommandBase {

  private final DrivebaseSubsystem drivebaseSubsystem;
  private final Pose2d finalPose;

  private final AdvancedSwerveTrajectoryFollower follower;

  private final double observationDistance;
  private final double observationTime;

  Optional<PathPlannerTrajectory> trajectory = Optional.empty();
  Future<PathPlannerTrajectory> futureTrajectory;

  double generationTime;
  boolean hasObserved;

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

  private Future<PathPlannerTrajectory> asyncPathGen(
      PathConstraints constraints, PathPoint initialPoint, PathPoint finalPoint) {
    return Executors.newCachedThreadPool()
        .submit(() -> PathPlanner.generatePath(constraints, initialPoint, finalPoint));
  }

  private Future<PathPlannerTrajectory> createObservationTrajectory() {
    System.out.println("gen observation trajectory");
    hasObserved = true;
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

    return asyncPathGen(
        new PathConstraints(5, 1.5),
        initialPoint,
        // stop near the goal to read apriltags
        observationPoint);
  }

  private Future<PathPlannerTrajectory> createAdjustTrajectory() {
    System.out.println("gen adjust trajectory");
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

    return asyncPathGen(new PathConstraints(1, .5), initialPoint, finalPoint);
  }

  private double distanceBetween(Pose2d p1, Pose2d p2) {
    return p1.getTranslation().getDistance(p2.getTranslation());
  }

  private boolean finishedPath() {
    return trajectory.isPresent()
        && (Timer.getFPGATimestamp() - generationTime
            > (trajectory.get().getTotalTimeSeconds() + observationTime));
  }

  private boolean poseSatisfied() {
    return trajectory.isPresent()
        && AdvancedSwerveTrajectoryFollower.poseWithinErrorMarginOfTrajectoryFinalGoal(
            finalPose, trajectory.get());
  }

  private void startGeneratingNextTrajectory() {
    futureTrajectory =
        (distanceBetween(drivebaseSubsystem.getPose(), finalPose) < observationDistance
                || hasObserved)
            ? createAdjustTrajectory()
            : createObservationTrajectory();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasObserved = false;
    startGeneratingNextTrajectory();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // start the next trajectory if the generation is done
    if (!trajectory.isPresent() && futureTrajectory.isDone()) {
      try {
        System.out.println("received finished trajectory, driving");
        trajectory = Optional.of(futureTrajectory.get());
        generationTime = Timer.getFPGATimestamp();
        follower.follow(trajectory.get());
      } catch (ExecutionException | InterruptedException e) {
        e.printStackTrace();
        this.cancel();
      }
    }

    if (trajectory.isPresent()) {
      // print the distance to the final pose
      var fP =
          ((PathPlannerState)
              trajectory
                  .get()
                  // sample the final position using the time greater than total time behavior
                  .sample(trajectory.get().getTotalTimeSeconds() + 1));
      System.out.println(
          String.format(
              "xy err: %8f theta err: %8f trajectoryTime: %8f",
              drivebaseSubsystem
                  .getPose()
                  .getTranslation()
                  .getDistance(fP.poseMeters.getTranslation()),
              Util.relativeAngularDifference(
                  drivebaseSubsystem.getPose().getRotation(), fP.holonomicRotation),
              trajectory.get().getTotalTimeSeconds()));
    }

    if (finishedPath()) {
      if (poseSatisfied()) {
        this.cancel();
      }
      System.out.println("creating new trajectory");
      startGeneratingNextTrajectory();
      trajectory = Optional.empty();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    follower.cancel();
    System.out.println("canceling future thread");
    futureTrajectory.cancel(true);
    trajectory = Optional.empty();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedPath() && poseSatisfied();
  }
}
