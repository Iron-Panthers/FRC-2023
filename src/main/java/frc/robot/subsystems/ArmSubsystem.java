// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Arm.Thresholds;
import frc.util.Util;

/** Add your docs here. */
public class ArmSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // TO DO: ADD VOLTAGE LIMITERS!!!

  private final TalonFX angleMotor;
  private final PIDController angleController;
  private final CANCoder angleEncoder;
  private double targetAngleDegrees; // measured in degrees

  private final TalonFX extensionMotor;
  private final PIDController extensionController;
  private double targetExtensionInches = 0;

  // gross members for logging to shuffleboard motor powers
  private double extensionOutput = 0;
  private double angleOutput = 0;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

  // stator limits
  private LinearFilter filter;

  private double filterOutput;

  public static record ArmState(double angle, double extension) {}

  public ArmSubsystem() {

    filter = LinearFilter.movingAverage(35);

    this.angleMotor = new TalonFX(Arm.Ports.ARM_MOTOR_PORT);
    extensionMotor = new TalonFX(Arm.Ports.TELESCOPING_MOTOR_PORT);

    angleMotor.setNeutralMode(NeutralMode.Brake);
    extensionMotor.setNeutralMode(NeutralMode.Brake);

    extensionMotor.configFactoryDefault();
    extensionMotor.setInverted(false);
    angleMotor.setInverted(true);

    extensionMotor.configForwardSoftLimitThreshold(
        inchesLengthToTicks(Arm.Setpoints.Extensions.MAX_EXTENSION), 20); // this is the top
    extensionMotor.configReverseSoftLimitThreshold(
        inchesLengthToTicks(Arm.Setpoints.Extensions.MIN_EXTENSION),
        20); // this is the bottom limit

    extensionMotor.configForwardSoftLimitEnable(true, 20);
    extensionMotor.configReverseSoftLimitEnable(true, 20);

    angleController = new PIDController(.019, 0, 0);
    extensionController = new PIDController(0.17, 0, 0);
    angleController.setTolerance(Thresholds.Angles.EPSILON);
    extensionController.setTolerance(Thresholds.Extensions.EPSILON);

    angleEncoder = new CANCoder(Arm.Ports.ENCODER_PORT);

    angleEncoder.configFactoryDefault();

    angleEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    targetAngleDegrees = Arm.Setpoints.STOWED.angle();

    var config =
        new StatorCurrentLimitConfiguration(
            true /*enable*/,
            50 /* current limit */,
            5 /* threshold */,
            .1 /*time in seconds to trip*/);

    angleMotor.configStatorCurrentLimit(config);

    angleEncoder.configMagnetOffset(Arm.ANGULAR_OFFSET);

    angleEncoder.configSensorDirection(true);

    angleEncoder.setPositionToAbsolute(10); // ms

    extensionMotor.setSelectedSensorPosition(0);

    tab.addDouble("current angle", this::getAngle);
    tab.addDouble("Desired Angle", () -> targetAngleDegrees);
    tab.add("Angle Arm PID", angleController);
    tab.add("Telescoping Arm PID", extensionController);
    tab.addNumber("Current Extension", this::getCurrentExtensionInches);
    tab.addNumber("Target Extension", () -> targetExtensionInches);
    tab.addNumber("extension error", () -> targetExtensionInches - getCurrentExtensionInches());
    tab.addNumber("Telescoping PID Output", () -> extensionOutput);
    tab.addBoolean(
        "Current or Target Angle within Unsafe Threshold",
        this::currentOrTargetAnglePassesUnsafeRange);
    tab.addNumber("Arm Gravity Offset", this::computeArmGravityOffset);
    tab.addNumber("Angle Output", () -> angleOutput);
    tab.addNumber("Angle Error", () -> targetAngleDegrees - getAngle());
    tab.addBoolean("At target", this::atTarget);
    tab.addString("Current Mode", () -> mode.toString());
    tab.addNumber("Stator current", this.extensionMotor::getStatorCurrent);
    tab.addNumber("filtered stator current", () -> this.filterOutput);
  }

  public enum Modes {
    DRIVETOPOS,
    ZERO
  }

  // current mode
  private Modes mode = Modes.DRIVETOPOS;

  public Modes getMode() {
    return mode;
  }

  public void setZeroMode() {
    extensionMotor.configForwardSoftLimitEnable(false, 20);
    extensionMotor.configReverseSoftLimitEnable(false, 20);
    mode = Modes.ZERO;
    targetExtensionInches = Arm.Setpoints.Extensions.MIN_EXTENSION;
  }

  /* methods for angle arm control */
  public double getAngle() {
    return angleEncoder.getAbsolutePosition();
  }

  public void setTargetAngleDegrees(double targetAngleDegrees) {
    mode = Modes.DRIVETOPOS;
    this.targetAngleDegrees =
        MathUtil.clamp(
            targetAngleDegrees,
            -Arm.Thresholds.Angles.UPPER_ANGLE_LIMIT,
            Arm.Thresholds.Angles.UPPER_ANGLE_LIMIT);
  }

  public double getTargetAngleDegrees() {
    return targetAngleDegrees;
  }

  public double getTargetExtensionInches() {
    return targetExtensionInches;
  }

  /* methods for telescoping arm control */
  public void setTargetExtensionInches(double targetExtensionInches) {
    mode = Modes.DRIVETOPOS;
    this.targetExtensionInches =
        MathUtil.clamp(
            targetExtensionInches,
            Arm.Setpoints.Extensions.MIN_EXTENSION,
            Arm.Setpoints.Extensions.MAX_EXTENSION);
  }

  public double getCurrentExtensionInches() {
    return ticksLengthToInches(extensionMotor.getSelectedSensorPosition());
  }

  private static double inchesLengthToTicks(double extension) {
    return (extension / Arm.SPOOL_CIRCUMFERENCE) * Arm.TELESCOPING_ARM_GEAR_RATIO * Arm.TICKS;
  }

  private static double ticksLengthToInches(double ticks) {
    return ((ticks / Arm.TICKS) / Arm.TELESCOPING_ARM_GEAR_RATIO) * Arm.SPOOL_CIRCUMFERENCE;
  }

  public void setTargetPosition(double targetAngleDegrees, double targetExtensionInches) {
    setTargetAngleDegrees(targetAngleDegrees);
    setTargetExtensionInches(targetExtensionInches);
  }

  public void setTargetPosition(ArmState targetState) {
    setTargetPosition(targetState.angle, targetState.extension);
  }

  /* safety methods */
  private boolean withinAngleRange(double angle) {
    return angle < Arm.Thresholds.Angles.FORWARD_UNSAFE_EXTENSION_ANGLE_THRESHOLD
        && angle > Arm.Thresholds.Angles.BACKWARD_UNSAFE_EXTENSION_ANGLE_THRESHOLD;
  }

  private boolean currentOrTargetAnglePassesUnsafeRange() {
    // if the current angle is within the unsafe range
    return withinAngleRange(getAngle())
        ||
        // if the target angle is within the unsafe range, return true
        withinAngleRange(targetAngleDegrees)
        // if the signs are opposite, arm must pass through bottom
        || Math.signum(targetAngleDegrees) != Math.signum(getAngle());
  }

  // Add the gravity offset as a function of sine
  private double computeArmGravityOffset() {
    return Math.sin(Math.toRadians(getAngle())) * Arm.GRAVITY_CONTROL_PERCENT;
  }

  private boolean extensionIsRetracted() {
    return getCurrentExtensionInches() < Arm.Thresholds.Extensions.FULLY_RETRACTED_INCHES_THRESHOLD;
  }

  private double computeIntermediateAngleGoal() {
    if (!extensionIsRetracted() && currentOrTargetAnglePassesUnsafeRange()) {
      // set the intermediate angle to the lowest safe angle on the same side as we are currently on
      if (getAngle() < 0) {
        return Arm.Thresholds.Angles.BACKWARD_UNSAFE_EXTENSION_ANGLE_THRESHOLD;
      }
      return Arm.Thresholds.Angles.FORWARD_UNSAFE_EXTENSION_ANGLE_THRESHOLD;
    }
    return targetAngleDegrees;
  }

  private double computeIntermediateExtensionGoal() {
    if (currentOrTargetAnglePassesUnsafeRange()) {
      return Arm.Setpoints.Extensions.MIN_EXTENSION;
    }
    return targetExtensionInches;
  }

  public boolean atTarget() {
    return Util.epsilonEquals(getAngle(), targetAngleDegrees, Thresholds.Angles.EPSILON)
        && Util.epsilonEquals(
            getCurrentExtensionInches(), targetExtensionInches, Thresholds.Extensions.EPSILON);
    // && extensionController.atSetpoint()
    // && angleController.atSetpoint();
  }

  @Override
  public void periodic() {
    this.filterOutput = this.filter.calculate(this.extensionMotor.getStatorCurrent());

    switch (mode) {
      case DRIVETOPOS:
        driveToPosPeriodic();
        break;
      case ZERO:
        zeroPeriodic();
        break;
    }
  }

  public void zeroPeriodic() {
    angleMotor.set(ControlMode.PercentOutput, computeArmGravityOffset());
    extensionMotor.set(ControlMode.PercentOutput, Arm.ZERO_RETRACTION_PERCENT);
    if (filterOutput > Arm.EXTENSION_STATOR_LIMIT) {
      extensionMotor.setSelectedSensorPosition(0);
      mode = Modes.DRIVETOPOS;
      extensionMotor.configForwardSoftLimitEnable(true, 20);
      extensionMotor.configReverseSoftLimitEnable(true, 20);
    }
  }

  public void driveToPosPeriodic() {

    double currentAngle = getAngle();
    // Add the gravity offset as a function of sine
    final double armGravityOffset = computeArmGravityOffset();

    extensionOutput =
        extensionController.calculate(
            getCurrentExtensionInches(), computeIntermediateExtensionGoal());

    angleOutput = angleController.calculate(currentAngle, computeIntermediateAngleGoal());
    angleMotor.set(
        ControlMode.PercentOutput, MathUtil.clamp(angleOutput + armGravityOffset, -.7, .7));
    extensionMotor.set(ControlMode.PercentOutput, MathUtil.clamp(extensionOutput, -.5, .5));
  }
}
