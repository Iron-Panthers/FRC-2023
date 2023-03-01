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
        new ArrayList<>(
            PathPlanner.loadPathGroup(
                "1 cube + 2 cone + balance",
                maxVelocityMetersPerSecond,
                maxAccelerationMetersPerSecondSq));

    addCommands(
        new ScoreCommand(outtakeSubsystem, armSubsystem, Constants.ScoringSteps.Cone.HIGH),
        new FollowTrajectoryCommand(paths.get(0), true, drivebaseSubsystem),
        intake(),
        new FollowTrajectoryCommand(paths.get(1), false, drivebaseSubsystem),
        new ScoreCommand(outtakeSubsystem, armSubsystem, Constants.ScoringSteps.Cone.HIGH),
        new FollowTrajectoryCommand(paths.get(2), false, drivebaseSubsystem),
        intake(),
        new FollowTrajectoryCommand(paths.get(3), false, drivebaseSubsystem),
        new ScoreCommand(outtakeSubsystem, armSubsystem, Constants.ScoringSteps.Cone.HIGH),
        new FollowTrajectoryCommand(paths.get(4), false, drivebaseSubsystem));
  }

  private SequentialCommandGroup intake() {
    // Add more commands in the future...maybe to retract arm?
    return new SequentialCommandGroup(
        new ArmPositionCommand(armSubsystem, Constants.Arm.Setpoints.GROUND_INTAKE),
        new ForceOuttakeSubsystemModeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE)
            .raceWith(new WaitCommand(4)));
  }
}
