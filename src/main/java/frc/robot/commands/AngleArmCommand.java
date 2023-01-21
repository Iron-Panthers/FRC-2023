// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AngleArmCommand extends CommandBase {
  ArmSubsystem armSubsystem;
  double desiredAngle;
  /** Creates a new AngleArmCommand. */
  public AngleArmCommand(ArmSubsystem armSubsystem, double desiredAngle) {
    this.armSubsystem = armSubsystem;
    this.desiredAngle = desiredAngle;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setDesiredAngle(desiredAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.atSetpoint();
  }
}
