// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ForceOuttakeSubsystemModeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.util.NodeSelectorUtility.Height;
import frc.util.NodeSelectorUtility.NodeType;
import java.util.Map;

public class N2_1CubePlus1ConeEngage extends SequentialCommandGroup {
  /** Creates a new N2_1CubePlus1ConeEngage. */
  public N2_1CubePlus1ConeEngage(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      Map<String, Command> eventMap,
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem) {

    PathPlannerTrajectory path1 =
        PathPlanner.loadPath(
            "n2 cone pickup to n3", maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    addCommands(
        // score the cube (later)
        new FollowTrajectoryCommand(path1, true, drivebaseSubsystem)
            .deadlineWith(
                new SetZeroModeCommand(armSubsystem)
                    .andThen(
                        new ArmPositionCommand(armSubsystem, Arm.Setpoints.GROUND_INTAKE)
                            .raceWith(
                                new ForceOuttakeSubsystemModeCommand(
                                        outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE)
                                    .withTimeout(4.5))
                            .andThen(new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED)))),
        new ScoreCommand(
            outtakeSubsystem,
            armSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CONE.atHeight(Height.HIGH)))

        // new ScoreCommand(
        //       outtakeSubsystem,
        //       armSubsystem,
        //       Constants.SCORE_STEP_MAP.get(scoreType),
        //       jason.leftBumper())
        // new BalanceCommand(drivebaseSubsystem)
        );
  }
}
