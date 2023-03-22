// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Arm.Setpoints;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ForceOuttakeSubsystemModeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.SetOuttakeModeCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.util.NodeSelectorUtility.Height;
import frc.util.NodeSelectorUtility.NodeType;
import frc.util.pathing.LoadMirrorPath;
import java.util.List;
import java.util.function.Supplier;

public class N9_2ConePlus1CubeMobility extends SequentialCommandGroup {
  /** Creates a new N2MobilityEngage. */
  public N9_2ConePlus1CubeMobility(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem) {

    List<Supplier<PathPlannerTrajectory>> path =
        LoadMirrorPath.loadPathGroup(
            "n9 2cone + 1cube mobility",
            maxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSq);

    addCommands(
        new SetZeroModeCommand(armSubsystem)
            .raceWith(new SetOuttakeModeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE)),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CONE.atHeight(Height.HIGH))),
        new FollowTrajectoryCommand(path.get(0), true, drivebaseSubsystem)
            .alongWith(
                (new WaitCommand(1))
                    .andThen(new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED))),
        (new WaitCommand(4))
            .deadlineWith(
                new ScoreCommand(outtakeSubsystem, armSubsystem, Setpoints.GROUND_INTAKE)
                    .alongWith(
                        new ForceOuttakeSubsystemModeCommand(
                            outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE))),
        new FollowTrajectoryCommand(path.get(1), false, drivebaseSubsystem)
            .alongWith(
                (new WaitCommand(1))
                    .andThen(new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED))),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CUBE.atHeight(Height.HIGH))),
        new FollowTrajectoryCommand(path.get(2), false, drivebaseSubsystem)
            .alongWith(
                (new WaitCommand(1))
                    .andThen(new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED))),
        (new WaitCommand(4))
            .deadlineWith(
                new ScoreCommand(outtakeSubsystem, armSubsystem, Setpoints.GROUND_INTAKE)
                    .alongWith(
                        new ForceOuttakeSubsystemModeCommand(
                            outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE))),
        new FollowTrajectoryCommand(path.get(3), false, drivebaseSubsystem)
            .alongWith(
                (new WaitCommand(1))
                    .andThen(new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED))),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CONE.atHeight(Height.HIGH))));
  }
}
