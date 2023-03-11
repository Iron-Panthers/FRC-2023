// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.SetOuttakeModeCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.util.NodeSelectorUtility.Height;
import frc.util.NodeSelectorUtility.NodeType;

public class Red_N3_1ConePlusMobilityEngage extends SequentialCommandGroup {
  /** Creates a new N2MobilityEngage. */
  public Red_N3_1ConePlusMobilityEngage(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem) {

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "[red] n3 1cone + mobility engage",
            maxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSq);

    addCommands(
        new WaitCommand(3).deadlineWith( new SetOuttakeModeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE)
            .alongWith(new SetZeroModeCommand(armSubsystem))),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CONE.atHeight(Height.HIGH))),
        (new FollowTrajectoryCommand(path, true, drivebaseSubsystem))
            .alongWith(
                (new WaitCommand(1))
                    .andThen(new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED))),
        new BalanceCommand(drivebaseSubsystem));
  }
}
