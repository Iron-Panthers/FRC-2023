package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeModes;

public class ForceIntakeCommand extends CommandBase{
    
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private IntakeModes mode;

    public ForceIntakeCommand(IntakeSubsystem intakeSubsystem, IntakeModes mode) {

        this.intakeSubsystem = intakeSubsystem;
        this.mode = mode;

        addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize() {
        intakeSubsystem.lockMode();
        intakeSubsystem.setMode(mode);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.unlockMode();
        intakeSubsystem.setMode(IntakeModes.OFF);
    }
}
