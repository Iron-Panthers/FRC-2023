// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class TelescopingArmPositionCommand extends CommandBase {
  private ArmSubsystem armSubsystem;
  private double targetExtension;
  private boolean withinAngleRange;
  /** Creates a new TelescopingArmPositionCommand. */
  public TelescopingArmPositionCommand(ArmSubsystem subsystem, double targetExtension) {
    // Use addRequirements() here to declare subsystem dependencies.
    armSubsystem = subsystem;
    this.targetExtension = targetExtension;
    withinAngleRange = false;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setTargetExtensionInches(targetExtension);
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
    return withinAngleRange;
  }
}
