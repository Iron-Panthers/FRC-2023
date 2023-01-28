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
import frc.robot.subsystems.VisionSubsystem;
import frc.util.AdvancedSwerveTrajectoryFollower;
import frc.util.AsyncWorker;
import frc.util.AsyncWorker.Result;
import frc.util.Util;

public class DriveToPlaceCommand extends CommandBase {

  private final DrivebaseSubsystem drivebaseSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final Pose2d observationPose;
  private final Pose2d finalPose;

  private final AdvancedSwerveTrajectoryFollower follower;

  private final double observationTime;

  AsyncWorker trajectGenerator = new AsyncWorker();

  Result<PathPlannerTrajectory> trajectoryResult;

  double generationTime;
  boolean hasObserved;

  /** Creates a new DriveToPlaceCommand. */
  public DriveToPlaceCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      VisionSubsystem visionSubsystem,
      Pose2d observationPose,
      Pose2d finalPose,
      double observationTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.observationPose = observationPose;
    this.finalPose = finalPose;
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

  private Result<PathPlannerTrajectory> asyncPathGen(
      PathConstraints constraints, PathPoint initialPoint, PathPoint finalPoint) {
    return trajectGenerator.submit(
        () -> PathPlanner.generatePath(constraints, initialPoint, finalPoint));
  }

  private Result<PathPlannerTrajectory> createObservationTrajectory() {
    System.out.println("gen observation trajectory");
    hasObserved = true;

    var currentPose = drivebaseSubsystem.getPose();
    var initialPoint =
        PathPoint.fromCurrentHolonomicState(currentPose, drivebaseSubsystem.getChassisSpeeds());

    var cameraObservationOffset =
        visionSubsystem
            .getRobotAngleToPointClosestCameraAtTargetAngle(observationPose.getRotation())
            .orElse(new Rotation2d());

    var observationPoint =
        new PathPoint(
            // drive until we are .2 meter away from the final position
            observationPose.getTranslation(),
            // drive in a straight line to the final position
            straightLineAngle(currentPose.getTranslation(), observationPose.getTranslation()),
            // holonomic rotation should be the same as the final rotation to ensure tag visibility

            observationPose.getRotation().plus(cameraObservationOffset));

    return asyncPathGen(
        new PathConstraints(5, 1.5),
        initialPoint,
        // stop near the goal to read apriltags
        observationPoint);
  }

  private Result<PathPlannerTrajectory> createAdjustTrajectory() {
    System.out.println("gen adjust trajectory");
    var currentPose = drivebaseSubsystem.getPose();
    // not from in motion
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

  private boolean finishedPath() {
    var optTraject = trajectoryResult.get();
    return optTraject.isPresent()
        && ((Timer.getFPGATimestamp() - generationTime)
            > (optTraject.get().getTotalTimeSeconds() + observationTime));
  }

  private boolean poseWithinErrorMarginOfFinal(Pose2d currentPose) {
    return (
        // xy error
        currentPose.getTranslation().getDistance(finalPose.getTranslation())
            <= PoseEstimator.DRIVE_TO_POSE_XY_ERROR_MARGIN_METERS)
        && (
        // theta error
        Math.abs(Util.relativeAngularDifference(currentPose.getRotation(), finalPose.getRotation()))
            <= PoseEstimator.DRIVE_TO_POSE_THETA_ERROR_MARGIN_DEGREES);
  }

  private boolean poseSatisfied() {
    var optTraject = trajectoryResult.get();
    return optTraject.isPresent() && poseWithinErrorMarginOfFinal(drivebaseSubsystem.getPose());
  }

  private void startGeneratingNextTrajectory() {
    trajectoryResult = hasObserved ? createAdjustTrajectory() : createObservationTrajectory();

    // drive the trajectory when it is ready
    trajectoryResult.subscribe(
        trajectory -> {
          System.out.println("received finished trajectory, driving");
          generationTime = Timer.getFPGATimestamp();
          follower.follow(trajectory.get());
        });
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

    // trigger trajectory following when the trajectory is ready
    trajectGenerator.heartbeat();

    var optTrajectory = trajectoryResult.get();
    if (optTrajectory.isPresent()) {
      var trajectory = optTrajectory.get();
      // print the distance to the final pose
      var fP =
          ((PathPlannerState)
              trajectory
                  // sample the final position using the time greater than total time behavior
                  .sample(trajectory.getTotalTimeSeconds() + 1));
      System.out.println(
          String.format(
              "xy err: %8f theta err: %8f trajectoryTime: %8f",
              drivebaseSubsystem
                  .getPose()
                  .getTranslation()
                  .getDistance(fP.poseMeters.getTranslation()),
              Util.relativeAngularDifference(
                  drivebaseSubsystem.getPose().getRotation(), fP.holonomicRotation),
              trajectory.getTotalTimeSeconds()));
    }

    if (finishedPath()) {
      if (poseSatisfied()) {
        this.cancel();
      } else {
        System.out.println("creating new trajectory");
        startGeneratingNextTrajectory();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    follower.cancel();
    System.out.println("canceling future thread");
    trajectoryResult = null;
    trajectGenerator.purge();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedPath() && poseSatisfied();
  }
}
