package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeShootCommand extends SequentialCommandGroup {

    public IntakeShootCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier isButtonPressed) {

        addCommands(
            new IntakeCommand(intakeSubsystem, IntakeSubsystem.Modes.INTAKE_LOW)
                .until(() -> intakeSubsystem.isCubeInRange()),
            new IntakeCommand(intakeSubsystem, IntakeSubsystem.Modes.STOWED)
                .raceWith(new WaitCommand(1)),
            new IntakeCommand(intakeSubsystem, IntakeSubsystem.Modes.OUTTAKE)
        );
    }
}