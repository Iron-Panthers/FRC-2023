package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ForceOuttakeSubsystemModeCommand;
import frc.robot.commands.SetOuttakeModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import java.util.ArrayList;

public class OneCubePlusTwoConePlusBalance extends SequentialCommandGroup {

  private ArmSubsystem armSubsystem;
  private OuttakeSubsystem outtakeSubsystem;

  public OneCubePlusTwoConePlusBalance(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      DrivebaseSubsystem drivebaseSubsystem) {

    ArrayList<PathPlannerTrajectory> paths =
        new ArrayList<PathPlannerTrajectory>(
            PathPlanner.loadPathGroup(
                "1 cube + 2 cone + balance",
                maxVelocityMetersPerSecond,
                maxAccelerationMetersPerSecondSq));

    addCommands(
        scoreMid(),
        new FollowTrajectoryCommand(paths.get(0), true, drivebaseSubsystem).alongWith(extendArm()),
        intake(),
        new FollowTrajectoryCommand(paths.get(1), false, drivebaseSubsystem).alongWith(extendArm()),
        scoreMid(),
        new FollowTrajectoryCommand(paths.get(2), false, drivebaseSubsystem),
        intake(),
        new FollowTrajectoryCommand(paths.get(3), false, drivebaseSubsystem).alongWith(extendArm()),
        scoreMid(),
        new FollowTrajectoryCommand(paths.get(4), false, drivebaseSubsystem));
  }

  private ParallelRaceGroup extendArm() {
    return new ArmPositionCommand(
            armSubsystem, Arm.Setpoints.ScoreMid.ANGLE, Arm.Setpoints.ScoreMid.EXTENSION)
        .raceWith(new WaitCommand(3));
  }

  /** First moves arm up to the desired angle, drops the cone, and then brings the arm back down */
  private SequentialCommandGroup scoreMid() {
    return new SequentialCommandGroup(
        new SetOuttakeModeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.OFF),
        new ArmPositionCommand(
                armSubsystem,
                Arm.Setpoints.GroundIntake.ANGLE,
                Arm.Setpoints.Extensions.MIN_EXTENSION)
            .raceWith(new WaitCommand(3)));
  }

  private ParallelRaceGroup intake() {
    // Add more commands in the future...maybe to retract arm?
    return new ForceOuttakeSubsystemModeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE)
        .raceWith(new WaitCommand(3));
  }
}
