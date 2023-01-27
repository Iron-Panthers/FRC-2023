// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants.Outtake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
  /** The modes of the drivebase subsystem */
  public enum Modes{
    OPEN,
    CLAMP
}

  private Modes mode;
  private final TalonFX outtake;

  private PIDController pidController;

  private CANCoder encoder;

  public OuttakeSubsystem() {

    this.mode = Modes.OPEN;

    this.outtake = new TalonFX(Outtake.Ports.OUTTAKE_MOTOR);

    this.encoder = new CANCoder(Outtake.Ports.OUTTAKE_ENCODER);

    this.pidController = new PIDController(0.01, 0, 0);
    pidController.setTolerance(3);

    
  }

  public double getAngle () {
    return encoder.getAbsolutePosition();
  }

  /**
   * gets the current mode of the outtake subsystem state machine
   *
   * @return the current mode
   */
  public Modes getMode() {
    return mode;
  }

  public void setMode(Modes mode) {
    this.mode = mode;
    
  }


  public void setMotorToAngle(double desiredAngle) {
    double output = pidController.calculate(getAngle(), desiredAngle);

    outtake.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(output, -0.3, 0.3));

  }

  public void openPeriodic(){

    setMotorToAngle(Outtake.OPEN_ANGLE);

  }

  public void clampPeriodic(){

    setMotorToAngle(Outtake.CLAMP_ANGLE);

  }


  public void applyMode() {
    switch (mode) {
      case OPEN:
        openPeriodic();
        break;
      case CLAMP:
        clampPeriodic();
        break;
     
    }
  }


  @Override
  public void periodic() {

    applyMode();
    
  }
}