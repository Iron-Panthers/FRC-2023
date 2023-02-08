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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Outtake;
import java.util.Optional;

public class OuttakeSubsystem extends SubsystemBase {
  /** The modes of the drivebase subsystem */
  public enum Modes {
    OPEN(Optional.empty()),
    OPENING(Optional.of(Outtake.StatorCurrents.OPENING_FINISH)),
    CLOSE(Optional.of(Outtake.StatorCurrents.ENDING_FINISH)),
    HOLD(Optional.empty());

    public final Optional<Double> statorTransitionCurrent;

    private Modes(Optional<Double> statorTransitionCurrent) {
      this.statorTransitionCurrent = statorTransitionCurrent;
    }
  }

  private Modes mode;
  private final TalonFX outtake;

  private PIDController pidController;

  // private CANCoder encoder;

  private LinearFilter filter;

  private double filterOutput;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Claw");

  public OuttakeSubsystem() {

    this.mode = Modes.OPEN;

    this.outtake = new TalonFX(Outtake.Ports.OUTTAKE_MOTOR);

    this.outtake.setInverted(true);

    this.outtake.configVoltageCompSaturation(11);
    this.outtake.enableVoltageCompensation(true);

    this.outtake.setNeutralMode(NeutralMode.Brake);

    this.pidController = new PIDController(0.01, 0, 0);
    pidController.setTolerance(3);

    // FIXME: Change the tap rate to get a better average
    filter = LinearFilter.movingAverage(9);

    filterOutput = 0;

    tab.addNumber("Stator Current", this.outtake::getStatorCurrent);
    tab.addNumber("FilterOUtput", () -> this.filterOutput);
    tab.addNumber("Angle of motor", this::getAngle);
    tab.addString("Current Mode", () -> mode.toString());
    tab.addNumber("Motor power?", this.outtake::getMotorOutputPercent);
  }

  public double getAngle() {
    return outtake.getSelectedSensorPosition();
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

  public void setMotorToAngle(double desiredAngle) {
    double output = pidController.calculate(getAngle(), desiredAngle);

    outtake.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(output, -0.3, 0.3));
  }

  public void openPeriodic() {
    outtake.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void closePeriodic() {
    outtake.set(TalonFXControlMode.PercentOutput, 0.075);
  }

  public void holdPeriodic() {
    outtake.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void openingPeriodic() {
    outtake.set(TalonFXControlMode.PercentOutput, -0.275);
  }

  public boolean inStableState() {
    return this.mode == Modes.OPEN || this.mode == Modes.HOLD;
  }

  public void advanceMode() {

    if (mode.statorTransitionCurrent.isPresent()
        && filterOutput > mode.statorTransitionCurrent.get()) {
      switch (mode) {
        case OPENING:
          mode = Modes.OPEN;
          break;
        case CLOSE:
          mode = Modes.HOLD;
          break;
        case OPEN:
        case HOLD:
          break;
      }
    }
  }

  public void applyMode() {
    switch (mode) {
      case OPEN:
        openPeriodic();
        break;
      case CLOSE:
        closePeriodic();
        break;
      case HOLD:
        holdPeriodic();
        break;
      case OPENING:
        openingPeriodic();
        break;
    }
  }

  @Override
  public void periodic() {

    this.filterOutput = this.filter.calculate(this.outtake.getStatorCurrent());

    advanceMode();

    applyMode();
  }
}
