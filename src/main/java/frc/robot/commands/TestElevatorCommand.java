package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorTestSubsystem;

public class TestElevatorCommand extends CommandBase {
    
    private ElevatorTestSubsystem subsystem;
    private double power = 0;
    public TestElevatorCommand(ElevatorTestSubsystem subsys, double motorPower) {
        subsystem = subsys;

        power = motorPower;

        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        subsystem.setMotorPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setMotorPower(0);
    }

}
