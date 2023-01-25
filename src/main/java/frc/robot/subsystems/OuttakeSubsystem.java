// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
  /** The modes of the drivebase subsystem */
  public enum Modes{
    IDLE,
    INTAKE,
    HOLD,
    OUTTAKE
}
private Modes mode = Modes.IDLE;


  /** Creates a new DrivebaseSubsystem. */
  public OuttakeSubsystem() {
    
  }

  private boolean modeLocked = false;

private double timeOfModeTransition = Timer.getFPGATimestamp();
  /**
   * gets the current mode of the drivebase subsystem state machine
   *
   * @return the current mode
   */
  public Modes getMode() {
    return mode;
  }

  public boolean getModeLocked(){
    return modeLocked;
  }

  /**
   * Angles the swerve modules in a cross shape, to make the robot hard to push. This function sets
   * the state machine to defense mode, so it only needs to be called once
   */
  public void setIdleMode() {
    mode = Modes.IDLE;
  }

  public void setIntakeMode() {
    mode = Modes.INTAKE;
  }

  public void setHoldMode() {
    mode = Modes.HOLD;
  }

  public void setOuttakeMode() {
    mode = Modes.OUTTAKE;
  }


  public double timeSinceModeTransition() {
    return Timer.getFPGATimestamp() - timeOfModeTransition;
  }

  public void setModeIfTimeSinceTransitionGreaterThan(double duration, Modes mode) {
    if (timeSinceModeTransition() >= duration) {
      setMode(mode);
    }
  }


  /**
   * Updates the robot pose estimation for newly written module states. Should be called on every
   * periodic
   */
  private void odometryPeriodic() {
    
  }

  public void nextModePeriodic(){
    if(modeLocked) return;
    
    switch(mode){
        case IDLE:
        case HOLD:
          break;
        case INTAKE:
          setMode(Modes.HOLD);
          break;
        case OUTTAKE:
          setMode(Modes.IDLE);
          break;
    }
  }

  public void setMode(Modes mode){
    timeOfModeTransition = Timer.getFPGATimestamp();
    this.mode = mode;
    modeLocked = false;
  }

  public void setLockMode(Modes mode){
    setMode(mode);
    modeLocked = true;
  }

  public void unlockMode(){
    modeLocked = false;
  }
 

  /**
   * Based on the current Mode of the drivebase, perform the mode-specific logic such as writing
   * outputs (may vary per mode).
   *
   * @param mode The mode to use (should use the current mode value)
   */
  public void updateModules(Modes mode) {
    switch (mode) {
      case IDLE:
        idlePeriodic();
        break;
      case INTAKE:
        intakePeriodic();
        break;
      case HOLD:
        holdPeriodic();
        break;
      case OUTTAKE:
        outtakePeriodic();
        break;
    }
  }

  public void idlePeriodic(){

  }

  public void intakePeriodic(){

  }

  public void holdPeriodic(){

  }

  public void outtakePeriodic(){

  }

  @Override
  public void periodic() {

  }
}