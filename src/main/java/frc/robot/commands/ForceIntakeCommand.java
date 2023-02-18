package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeModes;

public class ForceIntakeCommand extends CommandBase{
    
    private IntakeSubsystem intakeSubsystem;

    private IntakeModes mode, endMode;

    public ForceIntakeCommand(IntakeSubsystem intakeSubsystem, IntakeModes mode, IntakeModes endMode) {

        this.intakeSubsystem = intakeSubsystem;
        this.mode = mode;
        this.endMode = endMode;

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
        intakeSubsystem.setMode(endMode);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
