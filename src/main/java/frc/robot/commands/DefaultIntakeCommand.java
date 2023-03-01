package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpindexerHopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeModes;
import frc.robot.subsystems.SpindexerHopperSubsystem.Modes;

public class DefaultIntakeCommand extends SequentialCommandGroup {

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, SpindexerHopperSubsystem spindexerSubsystem, BooleanSupplier buttonPressed) {

        addCommands(
            new IntakeCommand(intakeSubsystem, IntakeModes.DEPLOY), 
            new ForceIntakeCommand(intakeSubsystem, IntakeModes.INTAKE, IntakeModes.OFF)
                .until(() -> !buttonPressed.getAsBoolean()),
            new WaitCommand(0.4),
            new IntakeCommand(intakeSubsystem, IntakeModes.RETRACT),
            new StartSpindexerHopperCommand(spindexerSubsystem, Modes.IDLE)
        );
    }
}
