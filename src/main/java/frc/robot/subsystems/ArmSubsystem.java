// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ArmSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // TO DO: ADD VOLTAGE LIMITERS!!!

  private final TalonFX armMotor;

  private final PIDController pidController;
  private final CANCoder armEncoder;

  private double desiredAngle;

  public ArmSubsystem() {

    this.armMotor = new TalonFX(Arm.Ports.ARM_MOTOR_PORT);

    armMotor.setNeutralMode(NeutralMode.Brake);

    pidController = new PIDController(0.1, 0, 0);

    armEncoder = new CANCoder(Arm.Ports.ENCODER_PORT);

    armEncoder.configFactoryDefault();

    armEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    desiredAngle = Arm.Setpoints.STARTING_ANGLE;

    var config =
        new StatorCurrentLimitConfiguration(
            true /*enable*/,
            50 /* current limit */,
            5 /* threshold */,
            .1 /*time in seconds to trip*/);

    armMotor.configStatorCurrentLimit(config);

    armEncoder.configMagnetOffset(Arm.ANGULAR_OFFSET);

    armEncoder.setPositionToAbsolute(10); // ms
  }

  public double getAngle() {
    return armEncoder.getAbsolutePosition();
  }

  public void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  @Override
  public void periodic() {
    double currentAngle = getAngle();

    double output = pidController.calculate(currentAngle, desiredAngle);

    // Add the gravity offset as a function of cosine
    final double gravityOffset =
        Math.cos(Math.toRadians(currentAngle)) * Arm.GRAVITY_CONTROL_PERCENT;

    armMotor.set(
        TalonFXControlMode.PercentOutput, MathUtil.clamp(output + gravityOffset, -0.1, 0.1));
  }
}
