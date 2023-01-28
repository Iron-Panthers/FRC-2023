package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPlaceCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class IanDemoAutoSequence extends SequentialCommandGroup {
  private double maxVelocityMetersPerSecond;
  private double maxAccelerationMetersPerSecondSq;
  private DrivebaseSubsystem drivebaseSubsystem;

  private PathPlannerTrajectory loadPath(String letter) {
    return PathPlanner.loadPath(
        "ian auto " + letter, maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);
  }

  private FollowTrajectoryCommand followPath(String letter) {
    return new FollowTrajectoryCommand(loadPath(letter), false, drivebaseSubsystem);
  }

  public IanDemoAutoSequence(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      DrivebaseSubsystem drivebaseSubsystem,
      VisionSubsystem visionSubsystem) {

    this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
    this.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;
    this.drivebaseSubsystem = drivebaseSubsystem;

    DriveToPlaceCommand score =
        new DriveToPlaceCommand(
            drivebaseSubsystem, visionSubsystem, new Pose2d(1.8, .5, Rotation2d.fromDegrees(180)));

    addCommands(followPath("A"), score, followPath("B"), score, followPath("C"), score);
  }
}
