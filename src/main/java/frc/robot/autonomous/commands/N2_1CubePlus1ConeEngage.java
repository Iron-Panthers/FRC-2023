// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import java.util.List;
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

    List<PathPlannerTrajectory> paths =
        PathPlanner.loadPathGroup(
            "n2 cone pickup to n3", maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    // addCommands(
    //     // score the cube (later)
    //     new FollowPathWithEvents(
    //         new FollowTrajectoryCommand(paths.get(0), true, drivebaseSubsystem),
    //         paths.get(0).getMarkers(),
    //         eventMap),
    //     new ScoreCommand(
    //         outtakeSubsystem,
    //         armSubsystem,
    //         Constants.SCORE_STEP_MAP.get(NodeType.CONE.atHeight(Height.HIGH))),
    //     new FollowPathWithEvents(
    //         new FollowTrajectoryCommand(paths.get(1), drivebaseSubsystem),
    //         paths.get(1).getMarkers(),
    //         eventMap),
    //     new BalanceCommand(drivebaseSubsystem));
  }
}
