// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Lights;

public class RGBSubsystem extends SubsystemBase {
  public static class RGBColor {
    public final int r;
    public final int g;
    public final int b;

    public RGBColor(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  private final CANdle candle;
  /** Creates a new RGBSubsystem. */
  public RGBSubsystem() {
    candle = new CANdle(Lights.CANDLE_ID);
    // turn off the LEDs when the can chain fails
    candle.configLOSBehavior(true);
    candle.configLEDType(LEDStripType.GRB);

    // start the rainbow

    showPulseColor(Lights.Colors.PURPLE);
  }

  public void showPulseColor(RGBColor color) {
    candle.animate(new SingleFadeAnimation(color.r, color.g, color.b, 0, .7, Lights.NUM_LEDS));
  }

  public void showBounceColor(RGBColor color) {
    candle.animate(
        new LarsonAnimation(
            color.r,
            color.g,
            color.b,
            0,
            .5,
            Lights.NUM_LEDS,
            LarsonAnimation.BounceMode.Front,
            7));
  }

  public void showRainbow() {
    candle.animate(new RainbowAnimation(.2, .5, Lights.NUM_LEDS));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
