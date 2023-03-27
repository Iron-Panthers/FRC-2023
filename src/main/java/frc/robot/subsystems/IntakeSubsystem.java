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
import frc.robot.Constants.Config;
import frc.robot.Constants.Intake;
import frc.util.SmartBoard;
import frc.util.SmartBoard.Range;
import frc.util.Util;
import java.util.List;

public class IntakeSubsystem extends SubsystemBase {
  private double currentAngle;
  private double targetAngle;

  private double angleOutput;
  private double anglePidOutput;
  private double angleGravityOutput;

  private final TalonFX intakeMotor;
  private final TalonFX angleMotor;

  private Modes mode;

  private final PIDController angleController;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

  private final List<SmartBoard> smartBoards =
      List.of(
          new SmartBoard(
              tab,
              "gravity control percent",
              () -> Intake.GRAVITY_CONTROL_PERCENT,
              v -> {
                Intake.GRAVITY_CONTROL_PERCENT = v;
              },
              new Range(0, .2)));

  public static class IntakeDetails {
    public final double angle;
    public final double intakePower;

    public IntakeDetails(double angle, double intakePower) {
      this.angle = angle;
      this.intakePower = intakePower;
    }
  }

  public enum Modes {
    INTAKE(Intake.IntakeModes.INTAKE),
    OUTTAKE(Intake.IntakeModes.OUTTAKE),
    DOWN(Intake.IntakeModes.DOWN),
    STOWED(Intake.IntakeModes.STOWED);

    public final IntakeDetails intakeDetails;

    private Modes(IntakeDetails intakeDetails) {
      this.intakeDetails = intakeDetails;
    }
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

    angleController = new PIDController(0.003, 0, 0); // FIXME tune

    intakeMotor.setNeutralMode(NeutralMode.Brake);
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
      tab.addString("mode", () -> mode.toString());
    }
  }

  // Add the gravity offset as a function of sine
  private double computeArmGravityOffset() {
    return Math.cos(Math.toRadians(getCurrentAngleDegrees() + Intake.GRAVITY_ANGLE_OFFSET))
        * Intake.GRAVITY_CONTROL_PERCENT;
  }

  public double getCurrentTicks() {
    return angleMotor.getSelectedSensorPosition();
  }

  public void zeroAngleEncoder() {
    applyZeroConfig();
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

  public void setTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  public boolean atTargetAngle() {
    return Util.epsilonEquals(getCurrentAngleDegrees(), targetAngle, Intake.EPSILON);
  }

  public Modes getMode() {
    return mode;
  }

  public void setMode(Modes mode) {
    this.mode = mode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetAngle = mode.intakeDetails.angle;
    currentAngle = getCurrentAngleDegrees();
    anglePidOutput = angleController.calculate(currentAngle - targetAngle);
    angleGravityOutput = computeArmGravityOffset();
    angleOutput = MathUtil.clamp(anglePidOutput + angleGravityOutput, -.3, .3);
    angleMotor.set(TalonFXControlMode.PercentOutput, angleOutput);
    intakeMotor.set(TalonFXControlMode.PercentOutput, mode.intakeDetails.intakePower);

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      smartBoards.forEach(SmartBoard::poll);
    }
  }
}
