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
import frc.util.Util;

/** Add your docs here. */
public class ArmSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // TO DO: ADD VOLTAGE LIMITERS!!!

  private final TalonFX armAngleMotor;
  private final PIDController angleController;
  private final CANCoder armEncoder;
  private double desiredAngle; // measured in degrees
  private double currentAngle;
  private boolean withinAngleRange;

  private final TalonFX telescopingMotor;
  private final PIDController extensionController;
  private double extension; // measured in inches
  private double targetExtension;
  private double extensionOutput;

  private double angleOutput;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

  public ArmSubsystem() {

    this.armAngleMotor = new TalonFX(Arm.Ports.ARM_MOTOR_PORT);
    telescopingMotor = new TalonFX(Arm.Ports.TELESCOPING_MOTOR_PORT);

    armAngleMotor.setNeutralMode(NeutralMode.Brake);
    telescopingMotor.setNeutralMode(NeutralMode.Brake);

    telescopingMotor.configFactoryDefault();
    telescopingMotor.setInverted(true);
    armAngleMotor.setInverted(true);

    telescopingMotor.configForwardSoftLimitThreshold(
        inchesLengthToTicks(Arm.Setpoints.MAX_EXTENSION), 0); // this is the top limit
    telescopingMotor.configReverseSoftLimitThreshold(
        inchesLengthToTicks(Arm.Setpoints.MIN_EXTENSION), 0); // this is the bottom limit

    telescopingMotor.configForwardSoftLimitEnable(true, 0);
    telescopingMotor.configReverseSoftLimitEnable(true, 0);

    angleController = new PIDController(0.01, 0, 0.001);
    extensionController = new PIDController(0.08, 0, 0);

    armEncoder = new CANCoder(Arm.Ports.ENCODER_PORT);

    armEncoder.configFactoryDefault();

    armEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    desiredAngle = Arm.Setpoints.STARTING_ANGLE;
    currentAngle = armEncoder.getAbsolutePosition();
    extension = 0;
    targetExtension = 0;
    extensionOutput = 0;
    withinAngleRange = false;
    angleOutput = 0;

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
    tab.add("Angle Arm PID", angleController);
    tab.add("Telescoping Arm PID", extensionController);
    tab.addNumber("Current Extension", this::getCurrentExtension);
    tab.addNumber("Target Extension", () -> targetExtension);
    tab.addNumber("Telescoping PID Output", () -> extensionOutput);
    tab.addBoolean("Within Angle Range", () -> withinAngleRange);
    tab.addNumber("Gravity control", this::gravityOffset);
    tab.addNumber("Angle Output", () -> angleOutput);
    tab.addNumber("Angle Error", () -> Math.abs(desiredAngle - currentAngle));
  }

  /* methods for angle arm control */
  public double getAngle() {
    return armEncoder.getAbsolutePosition();
  }

  public void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = MathUtil.clamp(desiredAngle, -Arm.UPPER_ANGLE_LIMIT, Arm.UPPER_ANGLE_LIMIT);
  }

  public double getDesiredAngle() {
    return desiredAngle;
  }

  /* methods for telescoping arm control */
  public void setTargetExtension(double extension) {
    targetExtension = extension;
  }

  public double getCurrentExtension() {
    extension = ticksLengthToInches(telescopingMotor.getSelectedSensorPosition());
    return extension;
  }

  private static double inchesLengthToTicks(double extension) {
    return (extension / Arm.SPOOL_CIRCUMFERENCE) * Arm.TELESCOPING_ARM_GEAR_RATIO * Arm.TICKS;
  }

  private static double ticksLengthToInches(double ticks) {
    return ((ticks / Arm.TICKS) / Arm.TELESCOPING_ARM_GEAR_RATIO) * Arm.SPOOL_CIRCUMFERENCE;
  }

  public void setDesiredPosition(double angle, double extension) {
    desiredAngle = angle;
    targetExtension = extension;
  }

  /* saftey methods */
  public boolean withinAngleRange(double angle) {
    if (Math.abs(angle) < Arm.ANGLE_THRESHOLD) {
      withinAngleRange = true;
      return true;
    }
    withinAngleRange = false;
    return false;
  }

  public boolean targetPassesAngleRange() {
    if (withinAngleRange(desiredAngle)
        || (desiredAngle / currentAngle
            < 0)) { // if the signs are opposite, arm must pass through bottom
      return true;
    }
    return false;
  }

  // Add the gravity offset as a function of cosine
  public double gravityOffset() {
    return Math.sin(Math.toRadians(getAngle())) * Arm.GRAVITY_CONTROL_PERCENT;
  }

  private void moveArm(double armPower, double extensionPower) {
    armAngleMotor.set(TalonFXControlMode.PercentOutput, armPower);
    telescopingMotor.set(TalonFXControlMode.PercentOutput, extensionPower);
  }

  @Override
  public void periodic() {

    currentAngle = getAngle();

    // Add the gravity offset as a function of cosine
    final double gravityOffset = gravityOffset();

    double tempTargetExtension =
        withinAngleRange(currentAngle) || targetPassesAngleRange() ? 0 : targetExtension;

    double tempDesiredAngle =
        !Util.epsilonEquals(getCurrentExtension(), 0, 2)
                && (withinAngleRange(currentAngle) || targetPassesAngleRange())
            ? Math.copySign(Arm.ANGLE_THRESHOLD, getAngle())
            : desiredAngle;

    extensionOutput =
        MathUtil.clamp(
            extensionController.calculate(getCurrentExtension(), tempTargetExtension), -0.2, 0.2);

    angleOutput =
        MathUtil.clamp(angleController.calculate(currentAngle, tempDesiredAngle), -0.7, 0.7);

    if (Util.epsilonEquals(Math.abs(currentAngle), Arm.ANGLE_THRESHOLD, 5)
        && extension > 0.5) { // within lower angle limits while arm is extended
      // hold arm at current angle
      // retract telescoping arm

      moveArm(gravityOffset, -0.2);
    } else if (Math.abs(currentAngle) > Arm.UPPER_ANGLE_LIMIT) { // within upper angle limits
      moveArm(0, extensionOutput);
    } else {
      moveArm(gravityOffset + angleOutput, extensionOutput);
    }
  }
}
