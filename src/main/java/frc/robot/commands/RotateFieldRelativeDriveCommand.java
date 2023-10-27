package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class RotateFieldRelativeDriveCommand extends CommandBase {
    
    private DrivebaseSubsystem drivebaseSubsystem;

    public RotateFieldRelativeDriveCommand(DrivebaseSubsystem drivebaseSubsystem) {

        this.drivebaseSubsystem = drivebaseSubsystem;
        
        addRequirements(drivebaseSubsystem);
    }


    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
