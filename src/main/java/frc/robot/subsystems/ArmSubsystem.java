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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

/** Add your docs here. */
public class ArmSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // TO DO: ADD VOLTAGE LIMITERS!!!

  private final TalonFX armAngleMotor;
  private final PIDController angleController;
  private final CANCoder armEncoder;
  private double desiredAngle;

  private TalonFX telescopingMotor;
  private PIDController extensionController;
  private double extension; // measured in inches
  private double targetExtension;
  private double extensionOutput;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

  public ArmSubsystem() {

    this.armAngleMotor = new TalonFX(Arm.Ports.ARM_MOTOR_PORT);
    telescopingMotor = new TalonFX(Arm.Ports.TELESCOPING_MOTOR_PORT);

    armAngleMotor.setNeutralMode(NeutralMode.Brake);
    telescopingMotor.setNeutralMode(NeutralMode.Brake);

    telescopingMotor.configFactoryDefault();
    telescopingMotor.setInverted(true);

    telescopingMotor.configForwardSoftLimitThreshold(
        heightToTicks(Arm.Setpoints.MAX_EXTENSION), 0); // this is the top limit
        telescopingMotor.configReverseSoftLimitThreshold(
        heightToTicks(Arm.Setpoints.MIN_EXTENSION), 0); // this is the bottom limit

    telescopingMotor.configForwardSoftLimitEnable(true, 0);
    telescopingMotor.configReverseSoftLimitEnable(true, 0);

    angleController = new PIDController(0.1, 0, 0);
    extensionController = new PIDController(0.02, 0, 0);

    armEncoder = new CANCoder(Arm.Ports.ENCODER_PORT);

    armEncoder.configFactoryDefault();

    armEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    desiredAngle = Arm.Setpoints.STARTING_ANGLE;
    extension = 0;
    targetExtension = 0;
    extensionOutput = 0;

    var config =
        new StatorCurrentLimitConfiguration(
            true /*enable*/,
            50 /* current limit */,
            5 /* threshold */,
            .1 /*time in seconds to trip*/);

    armAngleMotor.configStatorCurrentLimit(config);

    armEncoder.configMagnetOffset(Arm.ANGULAR_OFFSET);

    armEncoder.setPositionToAbsolute(10); // ms

    telescopingMotor.setSelectedSensorPosition(0);

    tab.addDouble("current angle", this::getAngle);
    tab.addDouble("Desired Angle", () -> desiredAngle);
    tab.add("Telescoping Arm PID", extensionController);
    tab.addNumber("Current Extension", this::getCurrentExtension);
    tab.addNumber("Target Extension", () -> targetExtension);
    tab.addNumber("Percent Output", this::getPercentOutput);
    tab.addNumber("Telescoping PID Output", () -> extensionOutput);
    

  }

  /* methods for angle arm control */
  public double getAngle() {
    return armEncoder.getAbsolutePosition();
  }

  public void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  /* methods for telescoping arm control */
  private static double heightToTicks(double extension) {
    return (extension / Arm.SPOOL_CIRCUMFERENCE)
        * Arm.TELESCOPING_ARM_GEAR_RATIO
        * Arm.TICKS;
  }

  private static double ticksToHeight(double ticks) {
    return ((ticks / Arm.TICKS) / Arm.TELESCOPING_ARM_GEAR_RATIO)
        * Arm.SPOOL_CIRCUMFERENCE;
  }

  public void setTargetExtension(double extension) {
    targetExtension = extension;
  }

  public double getCurrentExtension() {
    extension = ticksToHeight(telescopingMotor.getSelectedSensorPosition());
    return extension;
  }

  public double getPercentOutput() {
    return telescopingMotor.getMotorOutputPercent();
  }


  @Override
  public void periodic() {

    extensionOutput =
        MathUtil.clamp(extensionController.calculate(getCurrentExtension(), targetExtension), -0.2, 0.2);
    
    telescopingMotor.set(TalonFXControlMode.PercentOutput, extensionOutput);
    

    double currentAngle = getAngle();

    double angleOutput = angleController.calculate(currentAngle, desiredAngle);

    // Add the gravity offset as a function of cosine
    final double gravityOffset =
        Math.cos(Math.toRadians(currentAngle)) * Arm.GRAVITY_CONTROL_PERCENT;

    armAngleMotor.set(
        TalonFXControlMode.PercentOutput, MathUtil.clamp(angleOutput + gravityOffset, -0.1, 0.1));
  }
}
