package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.DrivebaseSubsystem;

public class N1_Hybrid1ConePlus2ConePlusEngage extends SequentialCommandGroup {
  public N1_Hybrid1ConePlus2ConePlusEngage(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      DrivebaseSubsystem drivebaseSubsystem) {

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "n1 hybrid 1cone + 2cone + engage",
            maxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSq);

    addCommands(new FollowTrajectoryCommand(path, true, drivebaseSubsystem));
  }
}
