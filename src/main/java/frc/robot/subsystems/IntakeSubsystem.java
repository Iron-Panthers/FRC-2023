// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.util.Util;

public class IntakeSubsystem extends SubsystemBase {
  private double currentAngle;
  private double targetAngle;

  private final TalonFX intakeMotor;
  private final TalonFX angleMotor;

  private Modes mode;

  private final PIDController angleController;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

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
    DOWN(Intake.IntakeModes.DOWN),
    STOWED(Intake.IntakeModes.STOWED);

    public final IntakeDetails intakeDetails;

    private Modes(IntakeDetails intakeDetails) {
      this.intakeDetails = intakeDetails;
    }
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Intake.Ports.INTAKE_MOTOR_PORT);
    angleMotor = new TalonFX(Intake.Ports.ANGLE_MOTOR_PORT);

    currentAngle = 0;
    targetAngle = 0;

    mode = Modes.STOWED;

    angleController = new PIDController(0.01, 0, 0); // FIXME tune

    intakeMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.setNeutralMode(NeutralMode.Brake);

    angleMotor.configForwardSoftLimitThreshold(
        ticksToAngleDegrees(Intake.Setpoints.MIN_ANGLE), 20); // this is the top
    angleMotor.configReverseSoftLimitThreshold(
        ticksToAngleDegrees(Intake.Setpoints.MAX_ANGLE), 20); // this is the bottom limit

    angleMotor.configForwardSoftLimitEnable(true, 20);
    angleMotor.configReverseSoftLimitEnable(true, 20);

    tab.add("angle pid", angleController);
    tab.addNumber("current angle", this::getCurrentAngleDegrees);
    tab.addNumber("target angle", () -> targetAngle);
    tab.addString("mode", () -> mode.toString());
    tab.addNumber("angle ticks", angleMotor::getSelectedSensorPosition);
    tab.addNumber("intake power", intakeMotor::getMotorOutputVoltage);
    tab.addBoolean("at target angle", this::atTargetAngle);
  }

  public double getCurrentAngleTicks() {
    return angleMotor.getSelectedSensorPosition();
  }

  public double getCurrentAngleDegrees() {
    return (getCurrentAngleTicks() / 2048) * 360; // TODO add these to constants
  }

  private double ticksToAngleDegrees(double ticks) {
    return (ticks / 2048) * 360; // TODO add these to constants
  }

  private double degreesToAngleTicks(double degrees) {
    return (degrees / 360) * 2048;
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
    double angleOutput = angleController.calculate(currentAngle, targetAngle);
    angleMotor.set(TalonFXControlMode.PercentOutput, angleOutput);
    intakeMotor.set(TalonFXControlMode.PercentOutput, mode.intakeDetails.intakePower);
  }
}
