// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeModes;
import frc.robot.subsystems.IntakeSubsystem.IntakeModeDetails.StatorLimit;
import java.util.Optional;

public class IntakeSubsystem extends SubsystemBase {
  public static class IntakeModeDetails {
    public final double upperSpeed;
    public final double lowerSpeed;

    public static class StatorLimit {
      public final double currentAmps;
      public final double armingDurationSeconds;

      public StatorLimit(double currentAmps, double armDurationSeconds) {
        this.currentAmps = currentAmps;
        this.armingDurationSeconds = armDurationSeconds;
      }
    }

    public final double delayEndBySeconds;
    public final Optional<StatorLimit> statorLimit;

    public IntakeModeDetails(
        double upperSpeed,
        double lowerSpeed,
        double delayEndBySeconds,
        Optional<StatorLimit> statorLimitAmps) {
      this.lowerSpeed = lowerSpeed;
      this.upperSpeed = upperSpeed;
      this.delayEndBySeconds = delayEndBySeconds;
      this.statorLimit = statorLimitAmps;
    }

    public IntakeModeDetails() {
      this(0, 0, 0, Optional.empty());
    }

    public IntakeModeDetails setUpperSpeed(double upperSpeed) {
      return new IntakeModeDetails(upperSpeed, lowerSpeed, delayEndBySeconds, statorLimit);
    }

    public IntakeModeDetails setLowerSpeed(double lowerSpeed) {
      return new IntakeModeDetails(upperSpeed, lowerSpeed, delayEndBySeconds, statorLimit);
    }

    public IntakeModeDetails setSpeed(double speed) {
      return new IntakeModeDetails(speed, speed, delayEndBySeconds, statorLimit);
    }

    public IntakeModeDetails setDelayEndBySeconds(double delayEndBySeconds) {
      return new IntakeModeDetails(upperSpeed, lowerSpeed, delayEndBySeconds, statorLimit);
    }

    public IntakeModeDetails setStatorLimitAmps(
        double statorLimitAmps, double armingDurationSeconds) {
      return new IntakeModeDetails(
          upperSpeed,
          lowerSpeed,
          delayEndBySeconds,
          Optional.of(new StatorLimit(statorLimitAmps, armingDurationSeconds)));
    }
  }

  private TalonFX lower;
  private TalonFX upper;

  private Modes mode = Modes.OFF;
  private boolean modeLocked = false;
  private double lastTransitionTime = Timer.getFPGATimestamp();

  private void applySettings(TalonFX motor) {
    motor.configVoltageCompSaturation(11);
  }

  public IntakeSubsystem() {

    lower = new TalonFX(Constants.Intake.Ports.LOWER);
    upper = new TalonFX(Constants.Intake.Ports.UPPER);

    Shuffleboard.getTab("intake").addDouble("lower stator", lower::getStatorCurrent);
    Shuffleboard.getTab("intake").addDouble("upper stator", upper::getStatorCurrent);
    Shuffleboard.getTab("intake")
        .addDouble(
            "limit",
            () ->
                mode.intakeModeDetails.statorLimit.isEmpty()
                    ? 0
                    : mode.intakeModeDetails.statorLimit.get().currentAmps);

    applySettings(lower);
    applySettings(upper);

    lower.setInverted(true);
  }

  public enum Modes {
    INTAKE(IntakeModes.INTAKE),
    OUTTAKE(IntakeModes.OUTTAKE),
    HOLD(IntakeModes.HOLD),
    OFF(IntakeModes.OFF);

    public final IntakeModeDetails intakeModeDetails;

    Modes(IntakeModeDetails intakeModeDetails) {
      this.intakeModeDetails = intakeModeDetails;
    }
  }

  public void setModeLocked(boolean modeLocked) {
    this.modeLocked = modeLocked;
  }

  public void setMode(Modes mode) {
    if (mode != this.mode) {
      this.lastTransitionTime = Timer.getFPGATimestamp();
    }
    this.mode = mode;
  }

  private boolean modeFinished() {

    if (this.mode.intakeModeDetails.statorLimit.isPresent()) {
      StatorLimit statorLimit = this.mode.intakeModeDetails.statorLimit.get();
      double statorLimitAmps = statorLimit.currentAmps;
      double armingDurationSeconds = statorLimit.armingDurationSeconds;

      if ((Timer.getFPGATimestamp() >= (lastTransitionTime + armingDurationSeconds))
          && (this.upper.getStatorCurrent() >= statorLimitAmps
              || this.lower.getStatorCurrent() >= statorLimitAmps)) return true;
    }

    return !modeLocked
        && (Timer.getFPGATimestamp()
            >= (lastTransitionTime + this.mode.intakeModeDetails.delayEndBySeconds));
  }

  private Modes advanceMode() {
    if (!modeFinished()) return mode;
    switch (mode) {
      case INTAKE:
        return Modes.HOLD;
      case OUTTAKE:
        return Modes.OFF;
        // states that do not progress automatically
      case OFF:
      case HOLD:
        break;
    }
    return mode;
  }

  private void applyMode(IntakeModeDetails mode) {
    upper.set(TalonFXControlMode.PercentOutput, mode.upperSpeed);
    lower.set(TalonFXControlMode.PercentOutput, mode.lowerSpeed);
  }

  @Override
  public void periodic() {

    setMode(advanceMode());

    applyMode(mode.intakeModeDetails);
  }
}
