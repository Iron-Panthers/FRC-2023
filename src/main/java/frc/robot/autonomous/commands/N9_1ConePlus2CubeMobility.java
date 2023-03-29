// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.SetOuttakeModeCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem.Modes;
import frc.util.pathing.LoadMirrorPath;
import java.util.Map;
import java.util.function.Supplier;

public class N9_1ConePlus2CubeMobility extends SequentialCommandGroup {
  public N9_1ConePlus2CubeMobility(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      Map<String, Command> eventMap,
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem) {

    Supplier<PathPlannerTrajectory> path =
        LoadMirrorPath.loadPath(
            "n9 3cube hybrid", maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

    addCommands(
        new FollowPathWithEvents(
            new FollowTrajectoryCommand(path, true, drivebaseSubsystem),
            path.get().getMarkers(),
            eventMap),
        new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED),
        new SetOuttakeModeCommand(outtakeSubsystem, Modes.OFF),
        new SetZeroModeCommand(armSubsystem));
  }
}
