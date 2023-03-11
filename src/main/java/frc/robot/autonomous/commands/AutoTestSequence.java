package frc.robot.autonomous.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.pathing.LoadMirrorPath;

public class AutoTestSequence extends SequentialCommandGroup {
  public AutoTestSequence(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      DrivebaseSubsystem drivebaseSubsystem) {

    Supplier<PathPlannerTrajectory> path =
        LoadMirrorPath.loadPath(
            "auto test", maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    addCommands(new FollowTrajectoryCommand(path, true, drivebaseSubsystem));
  }
}
