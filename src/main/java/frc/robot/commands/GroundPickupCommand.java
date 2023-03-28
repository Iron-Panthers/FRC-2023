// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Arm.Setpoints;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundPickupCommand extends ParallelCommandGroup {
  /** Creates a new GroundPickupCommand. */
  public GroundPickupCommand(
      IntakeSubsystem intakeSubsystem,
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new IntakeCommand(intakeSubsystem, IntakeSubsystem.Modes.INTAKE).asProxy(),
        new ArmPositionCommand(armSubsystem, Setpoints.HANDOFF).asProxy(),
        (new SetOuttakeModeCommand(outtakeSubsystem, OuttakeSubsystem.Modes.INTAKE)
                .andThen(
                    new ArmPositionCommand(armSubsystem, Arm.Setpoints.STOWED)
                        .alongWith(
                            new IntakeCommand(intakeSubsystem, IntakeSubsystem.Modes.STOWED))))
            .asProxy());
  }
}
