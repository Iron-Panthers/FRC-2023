// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants.Outtake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class OuttakeSubsystem extends SubsystemBase {
  /** The modes of the drivebase subsystem */
  public enum Modes{
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

  //private CANCoder encoder;

  private LinearFilter filter;

  private double filterOutput;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Claw");

  public OuttakeSubsystem() {

    this.mode = Modes.OPEN;

    this.outtake = new TalonFX(Outtake.Ports.OUTTAKE_MOTOR);

    this.outtake.setInverted(true);
   
    this.outtake.configVoltageCompSaturation(11);
    this.outtake.enableVoltageCompensation(true);


    //this.encoder = new CANCoder(Outtake.Ports.OUTTAKE_ENCODER);

    this.pidController = new PIDController(0.01, 0, 0);
    pidController.setTolerance(3);

    // FIXME: Change the tap rate to get a better average
    filter = LinearFilter.movingAverage(10);

    filterOutput = 0;

    tab.addNumber("Stator current", ()-> outtake.getStatorCurrent());
    tab.addNumber("FilterOUtput", () -> this.filterOutput);
    tab.addNumber("Angle of motor", this::getAngle);
    tab.addString("Current Mode", () -> mode.toString());
    
  }

  public double getAngle () {
    return outtake.getSelectedSensorPosition();
    //return encoder.getAbsolutePosition();
  }

  /**
   * gets the current mode of the outtake subsystem state machine
   *
   * @return the current mode
   */
  public Modes getMode() {
    return mode;
  }

  public void setMode(Modes mode2) {
    this.mode = mode2;
    
  }


  public void setMotorToAngle(double desiredAngle) {
    double output = pidController.calculate(getAngle(), desiredAngle);

    outtake.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(output, -0.3, 0.3));

  }

  public void openPeriodic(){

    outtake.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void closePeriodic(){


    outtake.set(TalonFXControlMode.PercentOutput, 0.075);

  }

  public void holdPeriodic(){

    outtake.set(TalonFXControlMode.PercentOutput, 0.25);

  }

  public void openingPeriodic () {
    outtake.set(TalonFXControlMode.PercentOutput, -0.055);
  }


  public void advanceMode() {

    if(mode.statorTransitionCurrent.isPresent() && filterOutput > mode.statorTransitionCurrent.get()){
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