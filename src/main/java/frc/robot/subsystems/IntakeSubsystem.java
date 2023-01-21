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
import frc.robot.Constants.Intake.IntakeModes.IntakeMode;

public class IntakeSubsystem extends SubsystemBase {

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

    applySettings(lower);
    applySettings(upper);

    lower.setInverted(true);
  }

  public enum Modes {
    INTAKE(IntakeModes.INTAKE),
    OUTTAKE(IntakeModes.OUTTAKE),
    HOLD(IntakeModes.HOLD),
    OFF(IntakeModes.OFF);

    public final IntakeMode intakeMode;

    Modes(IntakeMode intakeMode) {
      this.intakeMode = intakeMode;
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
    return Timer.getFPGATimestamp()
        >= (lastTransitionTime + this.mode.intakeMode.delayEndBySeconds);
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

  private void applyMode(IntakeMode mode) {
    upper.set(TalonFXControlMode.PercentOutput, mode.upperSpeed);
    lower.set(TalonFXControlMode.PercentOutput, mode.lowerSpeed);
  }

  @Override
  public void periodic() {

    if (!modeLocked) {
      setMode(advanceMode());
    }

    applyMode(mode.intakeMode);
  }
}
