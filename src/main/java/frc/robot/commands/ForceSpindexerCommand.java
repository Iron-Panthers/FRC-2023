package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpindexerHopperSubsystem;

public class ForceSpindexerCommand extends CommandBase{
    
    private SpindexerHopperSubsystem intakeSubsystem;

    private SpindexerHopperSubsystem.Modes mode, endMode;

    public ForceSpindexerCommand(SpindexerHopperSubsystem intakeSubsystem, SpindexerHopperSubsystem.Modes mode, SpindexerHopperSubsystem.Modes endMode) {

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
