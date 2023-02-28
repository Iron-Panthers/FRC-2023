// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Outtake;
import java.util.Optional;

public class OuttakeSubsystem extends SubsystemBase {

  public static class OuttakeDetails {
    public final double motorPower;
    public final Optional<StatorLimit> statorLimit;
    public final Optional<Double> minTimeSeconds;

    public static class StatorLimit {
      public final double statorTransitionCurrent;

      public StatorLimit(double statorTransitionCurrent) {
        this.statorTransitionCurrent = statorTransitionCurrent;
      }
    }

    public OuttakeDetails(
        double motorPower, Optional<StatorLimit> statorLimit, Optional<Double> minTimeSeconds) {
      this.motorPower = motorPower;
      this.statorLimit = statorLimit;
      this.minTimeSeconds = minTimeSeconds;
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

    private boolean modeFinished(
        double filterOutput, boolean modeLocked, double lastTransitionTime) {
      boolean exceededTimeLimit =
          outtakeDetails.minTimeSeconds.isEmpty()
              || Timer.getFPGATimestamp() - lastTransitionTime
                  > outtakeDetails.minTimeSeconds.get();
      // if time likmit is exceedd and mode is not locked
      if (exceededTimeLimit && !modeLocked) return true;
      // if time limit is exceeded and stator limit is exceeded, even if locked
      if (exceededTimeLimit
          && outtakeDetails.statorLimit.isPresent()
          && outtakeDetails.statorLimit.get().statorTransitionCurrent <= filterOutput) return true;
      return false;
    }
  }

  private Modes mode;
  private double lastTransitionTime;
  private boolean modeLocked;

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

    lastTransitionTime = 0;

    modeLocked = false;

    // FIXME: Change the tap rate to get a better average
    filter = LinearFilter.movingAverage(30);

    filterOutput = 0;

    tab.addNumber("Stator Current", this.outtake::getStatorCurrent);
    tab.addNumber("Filter Output", () -> this.filterOutput);

    tab.addNumber("voltage", this.outtake::getMotorOutputVoltage);

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
    if (this.mode != mode) lastTransitionTime = Timer.getFPGATimestamp();
    this.mode = mode;
  }

  public void lockMode() {
    this.modeLocked = true;
  }

  public void unlockMode() {
    this.modeLocked = false;
  }

  public boolean inStableState() {
    return this.mode == Modes.HOLD || this.mode == Modes.OFF;
  }

  private Modes advanceMode() {
    if (!mode.modeFinished(filterOutput, modeLocked, lastTransitionTime)) return mode;

    switch (mode) {
      case INTAKE:
        return Modes.HOLD;
      case OUTTAKE:
        return Modes.OFF;
      case OFF:
      case HOLD:
      default:
        return mode;
    }
  }

  private void applyMode() {
    outtake.set(TalonFXControlMode.PercentOutput, mode.outtakeDetails.motorPower);
  }

  @Override
  public void periodic() {

    this.filterOutput = this.filter.calculate(this.outtake.getStatorCurrent());

    setMode(advanceMode());

    applyMode();
  }
}
