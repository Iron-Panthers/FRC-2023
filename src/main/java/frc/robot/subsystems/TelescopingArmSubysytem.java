// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants.TelescopingArm;


public class TelescopingArmSubysytem extends SubsystemBase {
  private TalonFX motor;
  private PIDController pidController;
  private double position; // measured in inches
  private double targetPosition;
  
  private final ShuffleboardTab tab = Shuffleboard.getTab("Telescoping Arm");
  /** Creates a new TelescopingArmSubysytem. */
  public TelescopingArmSubysytem() {
    motor = new TalonFX(0); // TODO find CANID and put in constants
    pidController = new PIDController(0.01, 0, 0);

    position = 0;
    targetPosition = 0;

    motor.configFactoryDefault(); // do we need this??

    motor.configForwardSoftLimitThreshold(
        heightToTicks(TelescopingArm.MIN_HEIGHT), 0); // this is the bottom limit, we stop AT the bottom
    motor.configReverseSoftLimitThreshold(
        heightToTicks(TelescopingArm.MAX_HEIGHT), 0); // this is the top limit, we stop at the very top

    motor.configForwardSoftLimitEnable(true, 0);
    motor.configReverseSoftLimitEnable(true, 0);

    motor.setNeutralMode(NeutralMode.Brake);

    tab.add("PID", pidController);
    tab.addNumber("Current Position", this::getCurrentPosition);
    tab.addNumber("Target Position", () -> targetPosition);
    
  }

  public static double heightToTicks(double height) {
    return height / TelescopingArm.SPOOL_CIRCUMFERENCE * TelescopingArm.GEAR_RATIO;
  }

  public static double ticksToHeight(double ticks) {
    return ticks / TelescopingArm.GEAR_RATIO * TelescopingArm.SPOOL_CIRCUMFERENCE;
  }

  public void setTargetPosition(double position) {
    targetPosition = position;
  }

  public void getCurrentPosition() {
    position = motor.getSelectedSensorPosition();
    return position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double pidOutput = MathUtil.clamp(pidController.calculate(position, targetPosition), -0.1, 0.1);
    motor.set(TalonFXControlMode.PercentOutput, pidOutput);
  }
}
