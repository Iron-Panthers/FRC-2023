package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ForceOuttakeSubsystemModeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.SetZeroModeCommand;
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
      DrivebaseSubsystem drivebaseSubsystem,
      ArmSubsystem armSubsystem,
      OuttakeSubsystem outtakeSubsystem) {

    this.armSubsystem = armSubsystem;
    this.outtakeSubsystem = outtakeSubsystem;

    ArrayList<PathPlannerTrajectory> paths =
        new ArrayList<>(
            PathPlanner.loadPathGroup(
                "1 cube + 2 cone + balance",
                maxVelocityMetersPerSecond,
                maxAccelerationMetersPerSecondSq));

    addCommands(
        new SetZeroModeCommand(armSubsystem),
        // new ScoreCommand(outtakeSubsystem, armSubsystem, Constants.ScoringSteps.Cone.HIGH),
        new FollowTrajectoryCommand(paths.get(0), true, drivebaseSubsystem).alongWith(intake(6)),
        new SetZeroModeCommand(armSubsystem),
        new ScoreCommand(outtakeSubsystem, armSubsystem, Constants.ScoringSteps.Cone.HIGH),
        new FollowTrajectoryCommand(paths.get(1), false, drivebaseSubsystem).alongWith(intake(9)),
        new SetZeroModeCommand(armSubsystem),
        new ScoreCommand(outtakeSubsystem, armSubsystem, Constants.ScoringSteps.Cone.HIGH),
        new FollowTrajectoryCommand(paths.get(2), false, drivebaseSubsystem)
            .alongWith(
                new WaitCommand(2)
                    .andThen(
                        new ArmPositionCommand(armSubsystem, Constants.Arm.Setpoints.STOWED))));
  }

  private SequentialCommandGroup intake(double delaySeconds) {
    return new SequentialCommandGroup(
        new ArmPositionCommand(armSubsystem, Constants.Arm.Setpoints.GROUND_INTAKE)
            .alongWith(
                new ForceOuttakeSubsystemModeCommand(
                    outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE))
            .raceWith(new WaitCommand(delaySeconds)),
        new ArmPositionCommand(armSubsystem, Constants.Arm.Setpoints.STOWED));
  }
}
