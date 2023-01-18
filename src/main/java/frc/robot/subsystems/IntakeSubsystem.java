// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX intakeLower;
  private TalonFX intakeUpper;
  private TalonFX placeLower;
  private TalonFX placeUpper;

  public IntakeSubsystem() {

    intakeLower = new TalonFX(Constants.Intake.Ports.intakeLower);
    intakeUpper = new TalonFX(Constants.Intake.Ports.intakeUpper);
    placeLower = new TalonFX(Constants.Intake.Ports.intakeLower);
    placeUpper = new TalonFX(Constants.Intake.Ports.intakeUpper);

    placeLower.setInverted(true);
  }

  public void setIntake(double power) {
    double clampedPower = MathUtil.clamp(power, -0.1, 0.1);

    intakeLower.set(TalonFXControlMode.PercentOutput, clampedPower);
    intakeUpper.set(TalonFXControlMode.PercentOutput, clampedPower);
  }

  public void setPlaceLower(double power) {
    double clampedPower = MathUtil.clamp(power, -0.1, 0.1);
    placeLower.set(TalonFXControlMode.PercentOutput, clampedPower);
  }

  public void setPlaceUpper(double power) {
    double clampedPower = MathUtil.clamp(power, -0.1, 0.1);
    placeUpper.set(TalonFXControlMode.PercentOutput, clampedPower);
  }
}

// During outtake first spin upper intake and eject towards each other so it sucks the balls back.
// Then spin 1st eject and shoot, then shoot 2nd eject and shoot
