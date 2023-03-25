// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathConstraints;
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
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.util.NodeSelectorUtility.Height;
import frc.util.NodeSelectorUtility.NodeType;
import frc.util.pathing.LoadMirrorPath;
import java.util.List;
import java.util.function.Supplier;

public class N1_2ConePlusMobility extends SequentialCommandGroup {
  public N1_2ConePlusMobility(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem) {

    List<Supplier<PathPlannerTrajectory>> paths =
        LoadMirrorPath.loadPathGroup(
            "n1 2cone + mobility engage",
            new PathConstraints(maxVelocityMetersPerSecond, 7),
            new PathConstraints(maxVelocityMetersPerSecond, 7),
            new PathConstraints(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq));

    addCommands(
        new SetZeroModeCommand(armSubsystem)
            .deadlineWith(
                new SetOuttakeModeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE)),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CONE.atHeight(Height.HIGH)),
            1),
        (new FollowTrajectoryCommand(paths.get(0), true, drivebaseSubsystem))
            .alongWith(
                (new WaitCommand(.7))
                    .andThen(new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED))),
        new FollowTrajectoryCommand(paths.get(1), drivebaseSubsystem)
            .andThen(
                new ScoreCommand(
                        outtakeSubsystem, armSubsystem, Setpoints.GROUND_INTAKE.subList(0, 2), 1)
                    .deadlineWith(
                        new ForceOuttakeSubsystemModeCommand(
                            outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE))),
        new FollowTrajectoryCommand(paths.get(2), drivebaseSubsystem)
            .alongWith(
                new ScoreCommand(
                        // skip bringing it into the body
                        outtakeSubsystem, armSubsystem, Setpoints.GROUND_INTAKE.subList(2, 3), 1)
                    .andThen(
                        new ArmPositionCommand(
                            armSubsystem,
                            new ArmState(102.5, Arm.Setpoints.Extensions.MIN_EXTENSION)))),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CONE.atHeight(Height.HIGH)),
            1));
  }
}