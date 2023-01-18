// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeManualCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private final Double power;
  private final Double ejectPower;

  /** Creates a new ElevatorCommand. */
  public IntakeManualCommand(IntakeSubsystem subsystem, Double power, Double ejectPower) {
    this.power = power;
    this.ejectPower = ejectPower;
    this.intakeSubsystem = subsystem;
    addRequirements(intakeSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("rate", rate);
    intakeSubsystem.setPlaceLower(ejectPower);
    intakeSubsystem.setPlaceUpper(ejectPower);
    intakeSubsystem.setIntake(-power);
    intakeSubsystem.setIntake(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntake(0);
    intakeSubsystem.setPlaceLower(0);
    intakeSubsystem.setPlaceUpper(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
