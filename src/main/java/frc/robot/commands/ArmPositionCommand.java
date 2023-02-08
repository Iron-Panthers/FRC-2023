// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPositionCommand extends CommandBase {
  private ArmSubsystem subsystem;
  private final double desiredAngle;
  private final double targetExtension;
  /** Creates a new ArmPositionCommand. */
  public ArmPositionCommand(ArmSubsystem subsystem, double desiredAngle, double targetExtension) {
    this.subsystem = subsystem;
    this.desiredAngle = desiredAngle;
    this.targetExtension = targetExtension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setDesiredPosition(desiredAngle, targetExtension);
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
    return false;
  }
}
