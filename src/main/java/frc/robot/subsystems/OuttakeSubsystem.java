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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Config;
import frc.robot.Constants.Lights.Colors;
import frc.robot.Constants.Outtake;
import frc.robot.subsystems.RGBSubsystem.MessagePriority;
import frc.robot.subsystems.RGBSubsystem.PatternTypes;
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
      if (!exceededTimeLimit) return false;
      if (outtakeDetails.statorLimit.isEmpty()) return !modeLocked;

      return outtakeDetails.statorLimit.get().statorTransitionCurrent <= filterOutput;
    }
  }

  private Modes mode;
  private double lastTransitionTime;
  private boolean modeLocked;

  private final TalonFX outtake;

  private LinearFilter filter;

  private double filterOutput;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Claw");

  private final Optional<RGBSubsystem> rgbSubsystem;

  public OuttakeSubsystem(Optional<RGBSubsystem> rgbSubsystem) {

    this.rgbSubsystem = rgbSubsystem;

    this.mode = Modes.OFF;

    this.outtake = new TalonFX(Outtake.Ports.OUTTAKE_MOTOR);
    outtake.setInverted(true);

    // this.outtake.setInverted(true);

    this.outtake.configVoltageCompSaturation(11);
    this.outtake.enableVoltageCompensation(false);

    this.outtake.setNeutralMode(NeutralMode.Brake);

    lastTransitionTime = 0;

    modeLocked = false;

    // FIXME: Change the tap rate to get a better average
    filter = LinearFilter.movingAverage(30);

    filterOutput = 0;

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      tab.addNumber("Stator Current", this.outtake::getStatorCurrent);
      tab.addNumber("Filter Output", () -> this.filterOutput);

      tab.addNumber("voltage", this.outtake::getMotorOutputVoltage);

      tab.addString("Current Mode", () -> mode.toString());

      tab.addBoolean("mode locked", () -> this.modeLocked);
    }
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
        if (rgbSubsystem.isPresent()) {
          var msg =
              rgbSubsystem
                  .get()
                  .showMessage(
                      Colors.WHITE, PatternTypes.PULSE, MessagePriority.C_INTAKE_STATE_CHANGE);
          CommandScheduler.getInstance()
              .schedule(new WaitCommand(.7).andThen(new InstantCommand(msg::expire)));
        }
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
