package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.clearInvocations;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import edu.wpi.first.math.Pair;
import frc.RobotTest;
import frc.robot.Constants.Lights;
import frc.robot.subsystems.RGBSubsystem.RGBColor;
import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.CountDownLatch;
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
    // doing mocks like this is required because the CANdle constructor calls JNI
    // methods that we can't mock, which touch sim stuff during testing that
    // we don't want to touch
    mockCandle = Mockito.mockConstruction(CANdle.class);
    rgbSubsystem = new RGBSubsystem();
    candle = mockCandle.constructed().get(0);
    // we aren't interested in tracking the setup invocations
    clearInvocations(candle);
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
            RGBSubsystem.MessagePriority.F_NODE_SELECTION_COLOR);
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
            RGBSubsystem.MessagePriority.F_NODE_SELECTION_COLOR);
    doPeriodic(1);

    var msg2 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.BOUNCE,
            RGBSubsystem.MessagePriority.A_CRITICAL_NETWORK_FAILURE);

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
            RGBSubsystem.MessagePriority.F_NODE_SELECTION_COLOR);
    doPeriodic(1);

    var msg2 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.BOUNCE,
            RGBSubsystem.MessagePriority.A_CRITICAL_NETWORK_FAILURE);
    doPeriodic(1);

    msg2.expire();
    doPeriodic(1);
    showedBounce(1);
    showedPulse(2);
    showedRainbow(1);
  }

  @RobotTest
  public void differentColorWithSamePatternIsDisplayed() {
    var msg1 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.PULSE,
            RGBSubsystem.MessagePriority.F_NODE_SELECTION_COLOR);

    doPeriodic(1);
    showedRainbow(0);
    showedPulse(1);

    doPeriodic(1);
    showedRainbow(0);
    showedPulse(1);

    msg1.expire();
    var msg2 =
        rgbSubsystem.showMessage(
            Lights.Colors.BLUE,
            RGBSubsystem.PatternTypes.PULSE,
            RGBSubsystem.MessagePriority.F_NODE_SELECTION_COLOR);
    doPeriodic(1);
    showedRainbow(0);
    showedPulse(2);
  }

  @RobotTest
  public void sameColorWithSamePatternIsNotRedisplayed() {
    var msg1 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.PULSE,
            RGBSubsystem.MessagePriority.F_NODE_SELECTION_COLOR);

    doPeriodic(1);
    showedRainbow(0);
    showedPulse(1);

    msg1.expire();
    var msg2 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.PULSE,
            RGBSubsystem.MessagePriority.F_NODE_SELECTION_COLOR);
    doPeriodic(1);
    showedRainbow(0);
    showedPulse(1);

    var msg3 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.PULSE,
            RGBSubsystem.MessagePriority.A_CRITICAL_NETWORK_FAILURE);

    doPeriodic(1);
    showedRainbow(0);
    showedPulse(1);

    msg3.expire();
    doPeriodic(1);
    showedRainbow(0);
    showedPulse(1);

    msg2.expire();
    doPeriodic(1);
    showedRainbow(1);
    showedPulse(1);
  }

  @RobotTest
  // evil and flaky test
  public void multithreadedWritesDoNotProduceCrash() {
    var msg1 =
        rgbSubsystem.showMessage(
            Lights.Colors.MINT,
            RGBSubsystem.PatternTypes.PULSE,
            RGBSubsystem.MessagePriority.G_MISSING_PHOTONVISION_CLIENTS);

    doPeriodic(1);
    showedRainbow(0);
    showedPulse(1);

    // we don't want to count the old invocations...
    clearInvocations(candle);

    // run the periodic in a separate thread, constantly
    var periodicThread =
        new Thread(
            () -> {
              rgbSubsystem.periodic();
              try {
                Thread.sleep(10);
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
            });

    periodicThread.start();

    // it is time to spawn a bunch of threads
    var threads = new ArrayList<Pair<Thread, CountDownLatch>>();
    final int NUM_THREADS = 100;
    var messagesWritten = new CountDownLatch(NUM_THREADS);
    var messagesExpired = new CountDownLatch(NUM_THREADS);
    for (int i = 0; i < NUM_THREADS; i++) {
      var latch = new CountDownLatch(1);
      final int c = i;
      var thread =
          new Thread(
              () -> {
                var msg =
                    rgbSubsystem.showMessage(
                        // nasty white color but it needs to be different from the other messages
                        new RGBColor(c, c, c),
                        RGBSubsystem.PatternTypes.PULSE,
                        RGBSubsystem.MessagePriority.A_CRITICAL_NETWORK_FAILURE);
                messagesWritten.countDown();
                try {
                  latch.await();
                  msg.expire();
                  messagesExpired.countDown();
                } catch (InterruptedException e) {
                  fail(e);
                }
              });
      thread.start();
      threads.add(new Pair<>(thread, latch));
    }

    // wait for all the threads to write messages
    try {
      messagesWritten.await();
    } catch (InterruptedException e) {
      fail(e);
    }

    // now expire all the messages, in a random order
    Collections.shuffle(threads);
    for (var thread : threads) {
      thread.getSecond().countDown();
    }

    // wait for all the threads to expire messages
    try {
      messagesExpired.await();
    } catch (InterruptedException e) {
      fail(e);
    }

    // stop the periodic thread
    periodicThread.interrupt();

    // if we made it this far, no errors were thrown.
  }
}
