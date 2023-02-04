// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopingArm;

public class TelescopingArmSubysytem extends SubsystemBase {
  private TalonFX motor;
  private PIDController pidController;
  private double position; // measured in inches
  private double targetPosition;
  private double pidOutput;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Telescoping Arm");
  /** Creates a new TelescopingArmSubysytem. */
  public TelescopingArmSubysytem() {
    motor = new TalonFX(TelescopingArm.MOTOR_PORT); // TODO find CANID and put in constants
    pidController = new PIDController(0.02, 0, 0);

    position = 0;
    targetPosition = 0;
    pidOutput = 0;

    motor.configFactoryDefault();

    motor.setInverted(true);

    motor.setSelectedSensorPosition(0);

    motor.configForwardSoftLimitThreshold(
        heightToTicks(TelescopingArm.MAX_EXTENSION), 0); // this is the top limit
    motor.configReverseSoftLimitThreshold(
        heightToTicks(TelescopingArm.MIN_EXTENSION), 0); // this is the bottom limit

    motor.configForwardSoftLimitEnable(true, 0);
    motor.configReverseSoftLimitEnable(true, 0);

    motor.setNeutralMode(NeutralMode.Brake);

    tab.add("PID", pidController);
    tab.addNumber("Current Position", this::getCurrentPosition);
    tab.addNumber("Target Position", () -> targetPosition);
    tab.addNumber("Percent Output", this::getPercentOutput);
    tab.addNumber("PID Output", () -> pidOutput);
  }

  public static double heightToTicks(double extension) {
    return (extension / TelescopingArm.SPOOL_CIRCUMFERENCE)
        * TelescopingArm.GEAR_RATIO
        * TelescopingArm.TICKS;
  }

  public static double ticksToHeight(double ticks) {
    return ((ticks / TelescopingArm.TICKS) / TelescopingArm.GEAR_RATIO)
        * TelescopingArm.SPOOL_CIRCUMFERENCE;
  }

  public void setTargetPosition(double position) {
    targetPosition = position;
  }

  public double getCurrentPosition() {
    position = ticksToHeight(motor.getSelectedSensorPosition());
    return position;
  }

  public double getPercentOutput() {
    return motor.getMotorOutputPercent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    pidOutput =
        MathUtil.clamp(pidController.calculate(getCurrentPosition(), targetPosition), -0.2, 0.2);
    motor.set(TalonFXControlMode.PercentOutput, pidOutput);
  }
}
