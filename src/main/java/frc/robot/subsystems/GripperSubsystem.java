// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;

  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    intakeMotor = new TalonFX(Constants.Outtake.Ports.OUTTAKE_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
