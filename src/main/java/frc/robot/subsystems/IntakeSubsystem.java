// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeModes;
import frc.robot.Constants.Intake.IntakeModes.IntakeMode;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX lower;
  private TalonFX upper;

  private Modes mode = Modes.OFF;
  private boolean modeLocked = false;

  public IntakeSubsystem() {

    lower = new TalonFX(Constants.Intake.Ports.LOWER);
    upper = new TalonFX(Constants.Intake.Ports.UPPER);

    lower.setInverted(true);
  }

  public enum Modes {
    INTAKE(IntakeModes.INTAKE), OUTTAKE(IntakeModes.OUTTAKE), HOLD(IntakeModes.HOLD), OFF(IntakeModes.OFF) ;

    public final IntakeMode intakeMode;
    Modes(IntakeMode intakeMode) {
      this.intakeMode = intakeMode;
    }
  }

  public void setModeLocked(boolean modeLocked) {
    this.modeLocked = modeLocked;
  }

  public void setMode(Modes mode) {
    this.mode = mode;
  }

  private Modes advanceMode() {
    switch(mode) {
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


    if (!modeLocked) {mode = advanceMode();}

    applyMode(mode.intakeMode);
  }
}