// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Config;
import frc.robot.Constants.Drive.AutoBalance;
import frc.robot.subsystems.DrivebaseSubsystem;

public class EngageCommand extends CommandBase {
  enum Mode {
    DOCKING,
    ENGAGING,
    DEFENSE
  }

  DrivebaseSubsystem drivebaseSubsystem;
  boolean exceededDockingThreshold = false;
  Mode currentMode = Mode.DOCKING;

  /** Creates a new EngageCommand. */
  public EngageCommand(DrivebaseSubsystem drivebaseSubsystem) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    addRequirements(drivebaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    exceededDockingThreshold = false;
    currentMode = Mode.DOCKING;
    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      ShuffleboardTab tab = Shuffleboard.getTab("EngageCommand");
      tab.addString("Mode", () -> currentMode.toString());
      tab.addDouble("Angle", () -> this.drivebaseSubsystem.getRollPitch().roll());
    }
  }

  private Mode advanceState() {
    var rollPitch = drivebaseSubsystem.getRollPitch();

    switch (currentMode) {
      case DOCKING -> {
        if (rollPitch.absRoll() > AutoBalance.DOCK_HORIZON_ANGLE_DEGREES) {
          exceededDockingThreshold = true;
        }
        drivebaseSubsystem.drive(new ChassisSpeeds(AutoBalance.DOCK_SPEED_METERS_PER_SECOND, 0, 0));
        return (exceededDockingThreshold
                && rollPitch.absRoll() < AutoBalance.DOCK_MIN_ANGLE_DEGREES)
            ? Mode.ENGAGING
            : Mode.DOCKING;
      }
      case ENGAGING -> {
        drivebaseSubsystem.drive(
            new ChassisSpeeds(
                Math.copySign(AutoBalance.ENGAGE_SPEED_METERS_PER_SECOND, rollPitch.roll()), 0, 0));
        return (rollPitch.absRoll() < AutoBalance.ENGAGE_MIN_ANGLE_DEGREES)
            ? Mode.DEFENSE
            : Mode.ENGAGING;
      }

      case DEFENSE -> {
        drivebaseSubsystem.setDefenseMode();
        return rollPitch.absRoll() > AutoBalance.ENGAGE_MIN_ANGLE_DEGREES
            ? Mode.ENGAGING
            : Mode.DEFENSE;
      }
      default -> {
        System.err.println("Engage command unknown mode: " + currentMode);
        throw new IllegalStateException();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentMode = advanceState();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.setDefenseMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
