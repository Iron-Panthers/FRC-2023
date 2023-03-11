// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.DrivebaseSubsystem;

public class N2_Engage extends SequentialCommandGroup {
  /** Creates a new N2Engage. */
  public N2_Engage(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      DrivebaseSubsystem drivebaseSubsystem) {

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "n2 engage", maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    addCommands(
        new FollowTrajectoryCommand(path, true, drivebaseSubsystem),
        new BalanceCommand(drivebaseSubsystem));
  }
}
