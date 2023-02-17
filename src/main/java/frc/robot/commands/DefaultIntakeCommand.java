package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeModes;

public class DefaultIntakeCommand extends SequentialCommandGroup {
    
    private IntakeSubsystem intakeSubsystem;

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem) {

        this.intakeSubsystem = intakeSubsystem;

        addCommands(new IntakeCommand(intakeSubsystem, IntakeModes.DEPLOY), new ForceIntakeCommand(intakeSubsystem, IntakeModes.INTAKE, IntakeModes.RETRACT));

    }

}
