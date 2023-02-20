// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RGB;

public class RGBSubsystem extends SubsystemBase {
  private final CANdle candle;
  /** Creates a new RGBSubsystem. */
  public RGBSubsystem() {
    candle = new CANdle(RGB.CANDLE_ID);
    // turn off the LEDs when the can chain fails
    candle.configLOSBehavior(true);
    candle.configLEDType(LEDStripType.RGB);

    // start the rainbow
    candle.animate(new RainbowAnimation(.2, .5, RGB.NUM_LEDS));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
