// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Config;
import frc.robot.Constants.Intake;
import frc.util.SmartBoard;
import frc.util.Util;
import java.util.List;
import java.util.Optional;

public class IntakeSubsystem extends SubsystemBase {
  private double currentAngle;
  private double targetAngle;
  private double lastPulseTime;
  private boolean onHighCycle;

  // logs
  private double angleOutput;
  private double anglePidOutput;
  private double angleGravityOutput;
  private double filteredCurrent;

  private final TalonFX intakeMotor;
  private final TalonFX angleMotor;

  private Modes mode;
  private ControlTypes controlType;

  private LinearFilter statorFilter = LinearFilter.movingAverage(30);

  private final PIDController angleController;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

  private final List<SmartBoard> smartBoards =
      List.of(
          // new SmartBoard(
          //     tab,
          //     "gravity control percent",
          //     () -> Intake.GRAVITY_CONTROL_PERCENT,
          //     v -> {
          //       Intake.GRAVITY_CONTROL_PERCENT = v;
          //     },
          //     new Range(0, .2))
          );

  public record IntakeDetails(
      double angle,
      Optional<Double> highAngle,
      Optional<Double> alternatingPeriod,
      double intakePower) {
    public static IntakeDetails simple(double angle, double intakePower) {
      return new IntakeDetails(angle, Optional.empty(), Optional.empty(), intakePower);
    }

    public static IntakeDetails alternating(
        double lowAngle, double highAngle, double alternatingPeriod, double intakePower) {
      return new IntakeDetails(
          lowAngle, Optional.of(highAngle), Optional.of(alternatingPeriod), intakePower);
    }
  }

  public enum Modes {
    INTAKE(Intake.IntakeModes.INTAKE),
    INTAKE_LOW(Intake.IntakeModes.INTAKE_LOW),
    OUTTAKE(Intake.IntakeModes.OUTTAKE),
    DOWN(Intake.IntakeModes.DOWN),
    STOWED(Intake.IntakeModes.STOWED),
    CLIMB(Intake.IntakeModes.CLIMB);

    public final IntakeDetails intakeDetails;

    private Modes(IntakeDetails intakeDetails) {
      this.intakeDetails = intakeDetails;
    }
  }

  public enum ControlTypes {
    MODE,
    ZEROING
  }

  private void applyZeroConfig() {
    angleMotor.configForwardSoftLimitThreshold(ticksToAngleDegrees(Intake.Setpoints.MAX_ANGLE));
    angleMotor.configReverseSoftLimitThreshold(ticksToAngleDegrees(Intake.Setpoints.MIN_ANGLE));

    angleMotor.configForwardSoftLimitEnable(false);
    angleMotor.configReverseSoftLimitEnable(false);

    angleMotor.setSelectedSensorPosition(0);
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Intake.Ports.INTAKE_MOTOR_PORT);
    angleMotor = new TalonFX(Intake.Ports.ANGLE_MOTOR_PORT);

    currentAngle = 0;
    targetAngle = 0;

    mode = Modes.STOWED;

    angleController = new PIDController(0.006, 0, 0.0003);

    intakeMotor.configFactoryDefault();
    angleMotor.configFactoryDefault();

    intakeMotor.setNeutralMode(NeutralMode.Coast);

    intakeMotor.enableVoltageCompensation(true);
    intakeMotor.configVoltageCompSaturation(11);

    angleMotor.enableVoltageCompensation(true);
    angleMotor.configVoltageCompSaturation(11);

    angleMotor.setNeutralMode(NeutralMode.Brake);

    applyZeroConfig();

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {

      tab.add("angle pid", angleController);
      tab.addNumber("current angle", this::getCurrentAngleDegrees);
      tab.addNumber("target angle", () -> targetAngle);
      tab.addNumber("angle ticks", angleMotor::getSelectedSensorPosition);
      tab.addBoolean("at target angle", this::atTargetAngle);
      tab.addNumber("intake power", intakeMotor::getMotorOutputVoltage);
      tab.addNumber("angle power", angleMotor::getMotorOutputPercent);
      tab.addNumber("angle pid output", () -> this.anglePidOutput);
      tab.addNumber("angle gravity output", () -> this.angleGravityOutput);
      tab.addNumber("angle output", () -> this.angleOutput);
      tab.addNumber(
          "angle ff degree", () -> getCurrentAngleDegrees() + Intake.GRAVITY_ANGLE_OFFSET);
      tab.addString("mode", () -> mode.toString());
      tab.addNumber("angle error", () -> targetAngle - getCurrentAngleDegrees());
      tab.addNumber("filtered current", () -> this.filteredCurrent);
    }
  }

  // Add the gravity offset as a function of sine
  private double computeArmGravityOffset() {
    return -Math.sin(Math.toRadians(getCurrentAngleDegrees() + Intake.GRAVITY_ANGLE_OFFSET))
        * Intake.GRAVITY_CONTROL_PERCENT;
  }

  private double getCurrentTicks() {
    return angleMotor.getSelectedSensorPosition();
  }

  public double getCurrentRotation() {
    return (getCurrentTicks() / Intake.TICKS) * Intake.GEAR_RATIO;
  }

  public double getCurrentAngleDegrees() {
    return getCurrentRotation() * Intake.DEGREES;
  }

  private double ticksToAngleDegrees(double ticks) {
    return (ticks / Intake.TICKS) * Intake.GEAR_RATIO * Intake.DEGREES;
  }

  private double angleDegreesToTicks(double degrees) {
    return (degrees / Intake.DEGREES) * Intake.TICKS / Intake.GEAR_RATIO;
  }

  public boolean atTargetAngle() {
    return Util.epsilonEquals(getCurrentAngleDegrees(), targetAngle, Intake.EPSILON);
  }

  public Modes getMode() {
    return mode;
  }

  public ControlTypes getControlType() {
    return controlType;
  }

  public void setMode(Modes mode) {
    if (mode == this.mode) return;
    this.mode = mode;
    controlType = ControlTypes.MODE;
    onHighCycle = false;
    if (mode.intakeDetails.highAngle.isPresent()) {
      lastPulseTime = Timer.getFPGATimestamp();
    }
  }

  public void startZeroing() {
    controlType = ControlTypes.ZEROING;
  }

  @Override
  public void periodic() {

    filteredCurrent = statorFilter.calculate(angleMotor.getStatorCurrent());

    if (controlType == ControlTypes.MODE) {

      if (mode.intakeDetails.highAngle.isPresent()
          && (Timer.getFPGATimestamp() - lastPulseTime)
              > mode.intakeDetails.alternatingPeriod.orElseGet(() -> .5)) {
        lastPulseTime = Timer.getFPGATimestamp();
        onHighCycle = !onHighCycle;
      }

      targetAngle =
          onHighCycle
              ? mode.intakeDetails.angle
              : mode.intakeDetails.highAngle.orElseGet(mode.intakeDetails::angle);
      currentAngle = getCurrentAngleDegrees();
      anglePidOutput = angleController.calculate(currentAngle - targetAngle);
      angleGravityOutput = computeArmGravityOffset();
      angleOutput = MathUtil.clamp(anglePidOutput + angleGravityOutput, -.3, .3);
      angleMotor.set(TalonFXControlMode.PercentOutput, angleOutput);
      intakeMotor.set(TalonFXControlMode.PercentOutput, mode.intakeDetails.intakePower);
    } else if (controlType == ControlTypes.ZEROING) {
      angleMotor.set(TalonFXControlMode.PercentOutput, Intake.ZERO_PERCENT);
      intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
      if (filteredCurrent > Intake.ZEROING_STATOR_LIMIT) {
        controlType = ControlTypes.MODE;
        applyZeroConfig();
        setMode(Modes.STOWED);
      }
    }

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      smartBoards.forEach(SmartBoard::poll);
    }
  }
}
