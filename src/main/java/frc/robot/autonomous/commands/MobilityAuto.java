// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPlaceCommand;
import frc.robot.commands.SetOuttakeModeCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import frc.util.pathing.RubenManueverGenerator;
import java.util.Optional;

public class MobilityAuto extends SequentialCommandGroup {
  /** Creates a new MobilityAuto. */
  public MobilityAuto(
      RubenManueverGenerator manueverGenerator,
      DrivebaseSubsystem drivebaseSubsystem,
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      RGBSubsystem rgbSubsystem,
      Pose2d finalPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveToPlaceCommand(
                drivebaseSubsystem,
                manueverGenerator,
                () -> finalPose,
                () -> 0,
                () -> 0,
                () -> false,
                Optional.of(rgbSubsystem),
                Optional.empty())
            .alongWith(new SetZeroModeCommand(armSubsystem))
            .alongWith(new SetOuttakeModeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.HOLD)));
  }
}
