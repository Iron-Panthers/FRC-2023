package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auto;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import java.util.ArrayList;
import java.util.Map;

public class OneCubePlusTwoConePlusBalance extends SequentialCommandGroup {

  public OneCubePlusTwoConePlusBalance(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      DrivebaseSubsystem drivebaseSubsystem,
      ArmSubsystem armSubsystem,
      OuttakeSubsystem outtakeSubsystem) {

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

    final Map<String, Command> eventMap =
        Auto.CREATE_EVENT_MAP(drivebaseSubsystem, armSubsystem, outtakeSubsystem);

    addCommands(
        new FollowPathWithEvents(
            new FollowTrajectoryCommand(pathGroup.get(0), true, drivebaseSubsystem),
            path.getMarkers(),
            eventMap),
        eventMap.get("scoreConeHigh"),
        new FollowPathWithEvents(
            new FollowTrajectoryCommand(pathGroup.get(1), false, drivebaseSubsystem),
            path.getMarkers(),
            eventMap),
        eventMap.get("scoreConeHigh"),
        new FollowPathWithEvents(
            new FollowTrajectoryCommand(pathGroup.get(2), false, drivebaseSubsystem),
            path.getMarkers(),
            eventMap));
  }
}
