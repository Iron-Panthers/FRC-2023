// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem.Modes;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;



public class OuttakeCommand extends CommandBase {
  private final OuttakeSubsystem outtakeSubsystem;
  private final Modes mode; 




  

  /** Creates a new OuttakeCommand. */
  public OuttakeCommand(OuttakeSubsystem outtakeSubsystem, Modes mode){
    this.outtakeSubsystem = outtakeSubsystem; 
    this.mode = mode; 

  }
   
  


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  
  }

  @Override
  public void initialize() {
      // TODO Auto-generated method stub
      outtakeSubsystem.setMode(mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
