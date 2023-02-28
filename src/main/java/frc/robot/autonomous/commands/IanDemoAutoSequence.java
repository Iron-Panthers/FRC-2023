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
import frc.util.pathing.RubenManueverGenerator;

public class IanDemoAutoSequence extends SequentialCommandGroup {
  private double maxVelocityMetersPerSecond;
  private double maxAccelerationMetersPerSecondSq;
  private DrivebaseSubsystem drivebaseSubsystem;
  private VisionSubsystem visionSubsystem;
  private RubenManueverGenerator manueverGenerator;

  private PathPlannerTrajectory loadPath(String letter) {
    return PathPlanner.loadPath(
        "ian auto " + letter, maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);
  }

  private FollowTrajectoryCommand followPath(String letter) {
    return new FollowTrajectoryCommand(loadPath(letter), false, drivebaseSubsystem);
  }

  private DriveToPlaceCommand score() {
    return new DriveToPlaceCommand(
        drivebaseSubsystem,
        visionSubsystem,
        manueverGenerator,
        new Pose2d(1.8, .5, Rotation2d.fromDegrees(180)));
  }

  public IanDemoAutoSequence(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      DrivebaseSubsystem drivebaseSubsystem,
      VisionSubsystem visionSubsystem,
      RubenManueverGenerator manueverGenerator) {

    this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
    this.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.manueverGenerator = manueverGenerator;

    addCommands(
        score(), followPath("A"), score(), followPath("B"), score(), followPath("C"), score());
  }
}
