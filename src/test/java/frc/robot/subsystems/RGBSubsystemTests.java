package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import frc.RobotTest;
import frc.robot.Constants.Lights;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.MockedConstruction;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class RGBSubsystemTests {
  private MockedConstruction<CANdle> mockCandle;
  private CANdle candle;

  private RGBSubsystem rgbSubsystem;

  @BeforeEach
  public void setup() {
    mockCandle = Mockito.mockConstruction(CANdle.class);
    rgbSubsystem = new RGBSubsystem();
    candle = mockCandle.constructed().get(0);
    when(candle.animate(any(Animation.class))).thenReturn(ErrorCode.OK);
  }

  @AfterEach
  public void teardown() {
    mockCandle.close();
  }

  @RobotTest
  public void mocksInjectCorrectly() {
    // verify that the mock was injected correctly
    assertNotNull(candle);
    assertEquals(ErrorCode.OK, candle.animate(new RainbowAnimation()));
  }

  private void doPeriodic(int times) {
    for (int i = 0; i < times; i++) {
      rgbSubsystem.periodic();
    }
  }

  private void showedRainbow(int times) {
    verify(candle, times(times)).animate(any(RainbowAnimation.class));
  }

  private void showedPulse(int times) {
    verify(candle, times(times)).animate(any(SingleFadeAnimation.class));
  }

  private void showedBounce(int times) {
    verify(candle, times(times)).animate(any(LarsonAnimation.class));
  }

  @RobotTest
  public void noMessagesDisplaysRainbow() {
    doPeriodic(5);
    showedRainbow(1);
  }

  @RobotTest
  public void showingMessageDisplaysMessage() {
    doPeriodic(5);
    showedRainbow(1);
    var msg1 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.PULSE,
            RGBSubsystem.MessagePriority.B_DRIVER_CONTROLLED_COLOR);
    doPeriodic(1);
    showedPulse(1);
    showedRainbow(1);

    doPeriodic(5);
    showedPulse(1);
    showedRainbow(1);
  }

  @RobotTest
  public void higherPriorityMessageOverridesLowPriorityMessage() {
    doPeriodic(5);
    showedRainbow(1);
    var msg1 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.PULSE,
            RGBSubsystem.MessagePriority.B_DRIVER_CONTROLLED_COLOR);
    doPeriodic(1);

    var msg2 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.BOUNCE,
            RGBSubsystem.MessagePriority.A_CRITICAL_NETWORK_INFORMATION);

    doPeriodic(1);
    showedBounce(1);
    showedPulse(1);
    showedRainbow(1);

    doPeriodic(5);
    showedBounce(1);
    showedPulse(1);
    showedRainbow(1);
  }

  @RobotTest
  public void expiringHigherPriorityMessageOverridesLowPriorityMessage() {
    doPeriodic(5);
    showedRainbow(1);
    var msg1 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.PULSE,
            RGBSubsystem.MessagePriority.B_DRIVER_CONTROLLED_COLOR);
    doPeriodic(1);

    var msg2 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.BOUNCE,
            RGBSubsystem.MessagePriority.A_CRITICAL_NETWORK_INFORMATION);

    msg2.expire();
    doPeriodic(1);
    showedBounce(1);
    showedPulse(2);
    showedRainbow(1);
  }
}
