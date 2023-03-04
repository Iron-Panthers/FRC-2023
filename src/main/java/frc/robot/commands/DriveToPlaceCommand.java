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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PoseEstimator;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.AdvancedSwerveTrajectoryFollower;
import frc.util.AsyncWorker;
import frc.util.AsyncWorker.Result;
import frc.util.Util;
import frc.util.pathing.RubenManueverGenerator;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class DriveToPlaceCommand extends CommandBase {

  private final DrivebaseSubsystem drivebaseSubsystem;
  private final RubenManueverGenerator manueverGenerator;

  private final Supplier<Pose2d> observationPose;
  private final Supplier<Pose2d> finalPose;

  private final AdvancedSwerveTrajectoryFollower follower;

  private final double observationTime;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final BooleanSupplier isRobotRelativeSupplier;

  AsyncWorker trajectGenerator = new AsyncWorker();

  Result<Optional<PathPlannerTrajectory>> trajectoryResult;

  double generationTime;
  boolean hasObserved;
  boolean hasStartedDrivingPath;

  /** Creates a new DriveToPlaceCommand. */
  public DriveToPlaceCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      RubenManueverGenerator manueverGenerator,
      Supplier<Pose2d> observationPose,
      Supplier<Pose2d> finalPose,
      double observationTime,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      BooleanSupplier isRobotRelativeRelativeSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.manueverGenerator = manueverGenerator;
    this.observationPose = observationPose;
    this.finalPose = finalPose;
    this.observationTime = observationTime;

    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.isRobotRelativeSupplier = isRobotRelativeRelativeSupplier;

    follower = drivebaseSubsystem.getFollower();

    addRequirements(drivebaseSubsystem);
  }

  /**
   * Creates a new DriveToPlaceCommand without an observation pose.
   *
   * @param drivebaseSubsystem The drivebase subsystem.
   * @param visionSubsystem The vision subsystem.
   * @param finalPose The final pose to put the robot in.
   */
  public DriveToPlaceCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      RubenManueverGenerator manueverGenerator,
      Supplier<Pose2d> finalPose,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      BooleanSupplier isRobotRelativeRelativeSupplier) {
    this(
        drivebaseSubsystem,
        manueverGenerator,
        finalPose,
        finalPose,
        0.1,
        translationXSupplier,
        translationYSupplier,
        isRobotRelativeRelativeSupplier);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasObserved = false;
    hasStartedDrivingPath = false;
    // generation time will always be set before it is read as a product of only being read when
    // there is a trajectory and being written when one is generated
    startGeneratingNextTrajectory();
  }

  private Rotation2d straightLineAngle(Translation2d start, Translation2d end) {
    double x1 = start.getX();
    double y1 = start.getY();
    double x2 = end.getX();
    double y2 = end.getY();

    double angle = Math.atan2(y2 - y1, x2 - x1);
    return Rotation2d.fromRadians(angle);
  }

  private Result<Optional<PathPlannerTrajectory>> asyncPathGen(
      PathConstraints constraints, PathPoint initialPoint, PathPoint finalPoint) {
    return trajectGenerator.submit(
        () -> Optional.of(PathPlanner.generatePath(constraints, initialPoint, finalPoint)));
  }

  private Result<Optional<PathPlannerTrajectory>> createObservationTrajectory() {
    System.out.println("gen observation trajectory");
    hasObserved = true;
    var currentPose = drivebaseSubsystem.getPose();

    return trajectGenerator.submit(
        () ->
            manueverGenerator.computePath(
                currentPose, observationPose.get(), new PathConstraints(5, 2)));
  }

  private Result<Optional<PathPlannerTrajectory>> createAdjustTrajectory() {
    System.out.println("gen adjust trajectory");
    var currentPose = drivebaseSubsystem.getPose();
    // not from in motion
    var initialPoint =
        new PathPoint(
            currentPose.getTranslation(),
            straightLineAngle(currentPose.getTranslation(), finalPose.get().getTranslation()),
            // holonomic rotation should start at our current rotation
            currentPose.getRotation());

    var finalPoint =
        new PathPoint(
            // drive into the final position
            finalPose.get().getTranslation(),
            straightLineAngle(finalPose.get().getTranslation(), currentPose.getTranslation()),
            finalPose.get().getRotation());

    return asyncPathGen(new PathConstraints(5, 3), initialPoint, finalPoint);
  }

  private boolean finishedPath() {
    return trajectoryResult
        .get()
        .flatMap(Function.identity())
        .map(
            trajectory ->
                (Timer.getFPGATimestamp() - generationTime)
                    > (trajectory.getTotalTimeSeconds() + observationTime))
        .orElse(false);
  }

  private boolean poseWithinErrorMarginOfFinal(Pose2d currentPose) {
    return (
        // xy error
        currentPose.getTranslation().getDistance(finalPose.get().getTranslation())
            <= PoseEstimator.DRIVE_TO_POSE_XY_ERROR_MARGIN_METERS)
        && (
        // theta error
        Math.abs(
                Util.relativeAngularDifference(
                    currentPose.getRotation(), finalPose.get().getRotation()))
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
          if (trajectory.isEmpty() || trajectory.get().isEmpty()) {
            System.out.println("trajectory is empty");
            this.cancel();
            return;
          }
          System.out.println("received finished trajectory, driving");
          generationTime = Timer.getFPGATimestamp();
          follower.follow(trajectory.get().get());
          hasStartedDrivingPath = true;
        });
  }

  private static void trajectoryHealthDebugPrint(
      PathPlannerTrajectory trajectory, Pose2d currentPose) {
    // print distance to final pose
    var fP =
        ((PathPlannerState)
            trajectory
                // sample the final position using the time greater than total time behavior
                .sample(trajectory.getTotalTimeSeconds() + 1));
    System.out.println(
        String.format(
            "xy err: %8f theta err: %8f trajectoryTime: %8f",
            currentPose.getTranslation().getDistance(fP.poseMeters.getTranslation()),
            Util.relativeAngularDifference(currentPose.getRotation(), fP.holonomicRotation),
            trajectory.getTotalTimeSeconds()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // trigger trajectory following when the trajectory is ready
    trajectGenerator.heartbeat();

    // if we haven't driven a trajectory yet, let the driver keep driving
    // check after heartbeat to avoid doing both
    if (!hasStartedDrivingPath) {
      double x = translationXSupplier.getAsDouble();
      double y = translationYSupplier.getAsDouble();

      drivebaseSubsystem.drive(
          isRobotRelativeSupplier.getAsBoolean()
              ? new ChassisSpeeds(x, y, 0)
              : ChassisSpeeds.fromFieldRelativeSpeeds(
                  x, y, 0, drivebaseSubsystem.getDriverGyroscopeRotation()));
    }

    trajectoryResult
        .get()
        .flatMap(Function.identity())
        .ifPresent(
            trajectory -> trajectoryHealthDebugPrint(trajectory, drivebaseSubsystem.getPose()));

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
    // we null the result out for gc
    trajectoryResult = null;
    trajectGenerator.purge();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedPath() && poseSatisfied();
  }
}
