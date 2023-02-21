package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ForceOuttakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class BrandonAutoTestSequence extends SequentialCommandGroup {

  private double maxVelocityMetersPerSecond;
  private double maxAccelerationMetersPerSecondSq;
  private DrivebaseSubsystem drivebaseSubsystem;
  private ArmSubsystem armSubsystem;
  private OuttakeSubsystem outtakeSubsystem;

  public BrandonAutoTestSequence(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      DrivebaseSubsystem drivebaseSubsystem,
      ArmSubsystem armSubsystem,
      OuttakeSubsystem outtakeSubsystem) {

    this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
    this.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.armSubsystem = armSubsystem;
    this.outtakeSubsystem = outtakeSubsystem;

    //     //     new ArmPositionCommand(
    //       armSubsystem,
    //       Arm.Setpoints.GroundIntake.ANGLE,
    //       Arm.Setpoints.GroundIntake.EXTENSION))
    // .whileTrue(new ForceOuttakeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE));
    addCommands(
        scoreMid(),
        followPath("A", true),
        new ForceOuttakeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE)
            .raceWith(new WaitCommand(10)),
        followPath("B", false),
        scoreMid(),
        followPath("C", false));
  }

  private PathPlannerTrajectory loadPath(String letter) {
    return PathPlanner.loadPath(
        "brandon auto " + letter, maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);
  }

  private FollowTrajectoryCommand followPath(String letter, boolean localizeToStartPose) {
    return new FollowTrajectoryCommand(loadPath(letter), localizeToStartPose, drivebaseSubsystem);
  }

  /** First moves arm up to the desired angle, drops the cone, and then brings the arm back down */
  private SequentialCommandGroup scoreMid() {
    return new SequentialCommandGroup(
        new ArmPositionCommand(
                armSubsystem, Arm.Setpoints.ScoreMid.ANGLE, Arm.Setpoints.ScoreMid.EXTENSION)
            .raceWith(new WaitCommand(10)),
        new OuttakeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.OFF),
        new ArmPositionCommand(
                armSubsystem,
                Arm.Setpoints.GroundIntake.ANGLE,
                Arm.Setpoints.GroundIntake.EXTENSION)
            .raceWith(new WaitCommand(10)));
  }
}
