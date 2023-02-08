// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Outtake;
import java.util.Optional;

public class OuttakeSubsystem extends SubsystemBase {

  public static class OuttakeDetails {
    public final double motorPower;
    public final Optional<StatorLimit> statorLimit;

    public static class StatorLimit {
      public final double statorTransitionCurrent;
      public final boolean transitionWhenAboveThresholdCurrent;

      public StatorLimit(
          double statorTransitionCurrent, boolean transitionWhenAboveThresholdCurrent) {
        this.statorTransitionCurrent = statorTransitionCurrent;
        this.transitionWhenAboveThresholdCurrent = transitionWhenAboveThresholdCurrent;
      }
    }

    public OuttakeDetails(double motorPower, Optional<StatorLimit> statorLimit) {
      this.motorPower = motorPower;
      this.statorLimit = statorLimit;
    }

    public OuttakeDetails stableState(double motorPower) {
      return new OuttakeDetails(motorPower, Optional.empty());
    }
  }

  /** The modes of the drivebase subsystem */
  public enum Modes {
    HOLD(Outtake.OuttakeModes.HOLD),
    INTAKE(Outtake.OuttakeModes.INTAKE),
    OUTTAKE(Outtake.OuttakeModes.OUTTAKE),
    OFF(Outtake.OuttakeModes.OFF);

    public final OuttakeDetails outtakeDetails;

    private Modes(OuttakeDetails outtakeDetails) {
      this.outtakeDetails = outtakeDetails;
    }

    public Boolean modeFinished(double filterOutput) {
      // If there's no stator limit, then the mode is finished, and we can transition
      if (this.outtakeDetails.statorLimit.isEmpty()) return true;

      // If there's a stator transition current, get it
      double statorTransitionCurrent =
          this.outtakeDetails.statorLimit.get().statorTransitionCurrent;

      // Then, if we want transition when we're above the threshold current, we see if our output is
      // above the threshold
      // Otherwise, we can transition when we're below the threshold
      return this.outtakeDetails.statorLimit.get().transitionWhenAboveThresholdCurrent
          ? filterOutput > statorTransitionCurrent
          : filterOutput < statorTransitionCurrent;
    }
  }

  private Modes mode;
  private final TalonFX outtake;

  private LinearFilter filter;

  private double filterOutput;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Claw");

  public OuttakeSubsystem() {

    this.mode = Modes.OFF;

    this.outtake = new TalonFX(Outtake.Ports.OUTTAKE_MOTOR);

    // this.outtake.setInverted(true);

    this.outtake.configVoltageCompSaturation(11);
    this.outtake.enableVoltageCompensation(false);

    this.outtake.setNeutralMode(NeutralMode.Brake);

    // FIXME: Change the tap rate to get a better average
    filter = LinearFilter.movingAverage(90);

    filterOutput = 0;

    tab.addNumber("Stator Current", this.outtake::getStatorCurrent);
    tab.addNumber("Filter Output", () -> this.filterOutput);

    tab.addString("Current Mode", () -> mode.toString());
  }

  /**
   * gets the current mode of the outtake subsystem state machine
   *
   * @return the current mode
   */
  public Modes getMode() {
    return mode;
  }

  public void setMode(Modes mode) {
    this.mode = mode;
  }

  public Boolean modeFinished() {
    return this.mode.modeFinished(this.filterOutput);
  }

  public Boolean inStableState() {
    return this.mode == Modes.HOLD || this.mode == Modes.OFF;
  }

  public Modes advanceMode() {

    if (modeFinished()) {
      switch (mode) {
        case INTAKE:
          return Modes.HOLD;
        case OUTTAKE:
          return Modes.OFF;
        case OFF:
        case HOLD:
          return mode;
      }
    }
    return mode;
  }

  public void applyMode() {
    outtake.set(TalonFXControlMode.PercentOutput, mode.outtakeDetails.motorPower);
  }

  @Override
  public void periodic() {

    this.filterOutput = this.filter.calculate(this.outtake.getStatorCurrent());

    setMode(advanceMode());

    applyMode();
  }
}
