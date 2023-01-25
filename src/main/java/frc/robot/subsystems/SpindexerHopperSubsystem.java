// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.SpindexerHopper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerHopperSubsystem extends SubsystemBase {
  
  /** The modes of the SpindexerHopper subsystem */
  public enum Modes {
    IDLE,
    ALIGN,
    OFF
  }

  /** The current mode */
  private Modes mode = Modes.IDLE;

  private final TalonFX spinMotor;

  private double previousTimeOfTransition;

  private double transitionTime;

  /**
   * The Shuffleboard tab which all things related to the drivebase can be put for easy access and
   * organization
   */
  private final ShuffleboardTab tab = Shuffleboard.getTab("SpindexHopper");


  /** Creates a new SpindexHopper. */
  public SpindexerHopperSubsystem() {
  
    this.spinMotor = new TalonFX(SpindexerHopper.SPIN_MOTOR_PORT);

    spinMotor.setNeutralMode(NeutralMode.Brake);

    previousTimeOfTransition = 0;

    tab.addString("Current Mode", () -> mode.toString());

  }


  /**
   * gets the current mode of the SpindexerHopper subsystem state machine
   *
   * @return the current mode
   */
  public Modes getMode() {
    return mode;
  }

  /**
   * sets the current mode of the SpindexerHopper subsystem state machine
   */
  public void setMode(Modes mode) {
    this.mode = mode;
  }
  

  
  private void idlePeriodic() {
    spinMotor.set(ControlMode.PercentOutput, SpindexerHopper.IDLE_SPEED);

  }

  private void alignPeriodic() {
    spinMotor.set(ControlMode.PercentOutput, SpindexerHopper.ALIGN_SPEED);
  }

  private void offPeriodic() {
    spinMotor.set(ControlMode.PercentOutput, 0);
  }

  
  /**
   * Based on the current Mode of the drivebase, perform the mode-specific logic such as writing
   * outputs (may vary per mode).
   *
   * @param mode The mode to use (should use the current mode value)
   */
  public void applyMode(Modes mode) {
    switch (mode) {
      case IDLE:
        idlePeriodic();
        transitionTime = SpindexerHopper.IDLE_TO_ALIGN_TRANSITION_TIME;
        break;
      case ALIGN:
        alignPeriodic();
        transitionTime = SpindexerHopper.ALIGN_TO_OFF_TRANSITION_TIME;
        break;
      case OFF: 
        offPeriodic();
        transitionTime = 0;
    }
  }

  public Modes advanceMode(double currentTime, double previousTimeOfTransition, Modes mode) {
    
    if((currentTime - previousTimeOfTransition) >= transitionTime) {
      switch (mode) {
        case IDLE:
          return Modes.ALIGN;
        case ALIGN:
        case OFF:
          return Modes.OFF;   
        
      }

      this.previousTimeOfTransition = currentTime;
     }

    return mode;

  }


  @Override
  public void periodic() {

    setMode(advanceMode(Timer.getFPGATimestamp(), previousTimeOfTransition, mode));

    applyMode(mode);
    
  }
}
