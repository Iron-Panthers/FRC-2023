package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.util.NodeSelectorUtility.Height;
import frc.util.NodeSelectorUtility.NodeType;
import java.util.ArrayList;
import java.util.Map;

public class OneCubePlusTwoConePlusBalance extends SequentialCommandGroup {

  public OneCubePlusTwoConePlusBalance(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      DrivebaseSubsystem drivebaseSubsystem,
      ArmSubsystem armSubsystem,
      OuttakeSubsystem outtakeSubsystem,
      Map<String, Command> eventMap) {

    ArrayList<PathPlannerTrajectory> pathGroup =
        new ArrayList<>(
            PathPlanner.loadPathGroup(
                "1 cube + 2 cone + balance",
                maxVelocityMetersPerSecond,
                maxAccelerationMetersPerSecondSq));

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "1 cube + 2 cone + balance",
            maxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSq);

    addCommands(
        new FollowPathWithEvents(
            new FollowTrajectoryCommand(pathGroup.get(0), true, drivebaseSubsystem),
            pathGroup.get(0).getMarkers(),
            eventMap),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CONE.atHeight(Height.HIGH))),
        new ArmPositionCommand(armSubsystem, Constants.Arm.Setpoints.STOWED),
        new FollowPathWithEvents(
            new FollowTrajectoryCommand(pathGroup.get(1), false, drivebaseSubsystem),
            pathGroup.get(1).getMarkers(),
            eventMap),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CONE.atHeight(Height.HIGH))),
        new ArmPositionCommand(armSubsystem, Constants.Arm.Setpoints.STOWED),
        new FollowPathWithEvents(
            new FollowTrajectoryCommand(pathGroup.get(2), false, drivebaseSubsystem),
            pathGroup.get(2).getMarkers(),
            eventMap));
  }
}
