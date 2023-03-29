// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.SetOuttakeModeCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.commands.ZeroIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.util.NodeSelectorUtility.Height;
import frc.util.NodeSelectorUtility.NodeType;
import frc.util.pathing.LoadMirrorPath;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class N9_1ConePlus2CubeMobility extends SequentialCommandGroup {
  public N9_1ConePlus2CubeMobility(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      Map<String, Command> eventMap,
      IntakeSubsystem intakeSubsystem,
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem) {

    List<Supplier<PathPlannerTrajectory>> paths =
        LoadMirrorPath.loadPathGroup(
            "n9 1cone + 2cube", maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    addCommands(
        (new SetZeroModeCommand(armSubsystem).alongWith(new ZeroIntakeCommand(intakeSubsystem)))
            .deadlineWith(
                new SetOuttakeModeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE)),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CONE.atHeight(Height.HIGH)),
            1),
        new FollowPathWithEvents(
            new FollowTrajectoryCommand(paths.get(0), true, drivebaseSubsystem),
            paths.get(0).get().getMarkers(),
            eventMap),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CUBE.atHeight(Height.HIGH))),
        new FollowPathWithEvents(
            new FollowTrajectoryCommand(paths.get(1), drivebaseSubsystem),
            paths.get(1).get().getMarkers(),
            eventMap),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CUBE.atHeight(Height.MID))));
  }
}
