package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ForceOuttakeSubsystemModeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import java.util.HashMap;

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

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "1 cube + 2 cone + balance",
            maxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSq);

    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put("intake", intake());
    eventMap.put(
        "scoreConeHigh",
        new ScoreCommand(outtakeSubsystem, armSubsystem, Constants.ScoringSteps.Cone.HIGH));
    eventMap.put("stow", new ArmPositionCommand(armSubsystem, Constants.Arm.Setpoints.STOWED));
    eventMap.put("zeroArm", new SetZeroModeCommand(armSubsystem));

    // TODO: Actually get these features working lol
    eventMap.put("scoreCubeHigh", new InstantCommand());
    eventMap.put("balance", new InstantCommand());

    addCommands(
        new FollowPathWithEvents(
            new FollowTrajectoryCommand(path, true, drivebaseSubsystem),
            path.getMarkers(),
            eventMap));
  }

  private ParallelCommandGroup intake() {
    return new ArmPositionCommand(armSubsystem, Constants.Arm.Setpoints.GROUND_INTAKE)
        .alongWith(
            new ForceOuttakeSubsystemModeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE));
  }
}
