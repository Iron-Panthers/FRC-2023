package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DrivebaseSubsystem;

public class N1_Hybrid1ConePlus2ConePlusEngage extends SequentialCommandGroup {
  public N1_Hybrid1ConePlus2ConePlusEngage(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem) {

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "n1 hybrid 1cone + 2cone + engage",
            maxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSq);

    addCommands(
        new SequentialCommandGroup(
                new ArmPositionCommand(armSubsystem, new ArmState(-55, 0)),
                new SetZeroModeCommand(armSubsystem),
                new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED))
            .alongWith(new FollowTrajectoryCommand(path, true, drivebaseSubsystem)));
  }
}
