// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Lights;
import java.util.Objects;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.concurrent.ConcurrentLinkedQueue;

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

  public enum MessagePriority {
    A_CRITICAL_NETWORK_FAILURE,
    B_MISSING_CAN_DEVICE,
    C_INTAKE_STATE_CHANGE,
    D_PATHING_STATUS,
    E_DRIVER_CONTROLLED_COLOR,
    F_NODE_SELECTION_COLOR,
    G_MISSING_PHOTONVISION_CLIENTS,
    H_NETWORK_HEALTHY,
  }

  private enum CurrentAnimationTypes {
    RAINBOW,
    LARSON,
    SINGLE_FADE,
    STROBE
  }

  public enum PatternTypes {
    PULSE(CurrentAnimationTypes.SINGLE_FADE),
    BOUNCE(CurrentAnimationTypes.LARSON),
    STROBE(CurrentAnimationTypes.STROBE);

    private final CurrentAnimationTypes type;

    private PatternTypes(CurrentAnimationTypes type) {
      this.type = type;
    }
  }

  private Optional<CurrentAnimationTypes> lastAppliedAnimation = Optional.empty();
  private Optional<RGBColor> lastAppliedColor = Optional.empty();

  /** This class does not obey equals and compareTo contract to support priority ordering. */
  public static class RGBMessage implements Comparable<RGBMessage> {
    private final RGBColor color;
    private final PatternTypes pattern;
    private final MessagePriority priority;
    private boolean isExpired = false;

    private RGBMessage(RGBColor color, PatternTypes pattern, MessagePriority priority) {
      this.color = color;
      this.pattern = pattern;
      this.priority = priority;
    }

    public void expire() {
      isExpired = true;
    }

    public int compareTo(RGBMessage other) {
      return priority.compareTo(other.priority);
    }

    public boolean equals(Object other) {
      if (other instanceof RGBMessage) {
        var otherMessage = (RGBMessage) other;
        return priority.equals(otherMessage.priority)
            && isExpired == otherMessage.isExpired
            && color.equals(otherMessage.color)
            && pattern.equals(otherMessage.pattern);
      }
      return false;
    }

    public int hashCode() {
      return Objects.hash(color, pattern, priority, isExpired);
    }
  }

  private final CANdle candle;

  /**
   * The queue of messages that have been sent to the RGB subsystem, but not yet drained. Messages
   * are drained into the drained queue during periodic. This serves to achieve thread safety, even
   * when rgb messages are created from threads other than the main thread, without locking the main
   * thread when reading the message queue.
   */
  private final ConcurrentLinkedQueue<RGBMessage> threadSafeMessageSink =
      new ConcurrentLinkedQueue<>();

  /**
   * The priority queue of messages to display. Messages with higher priority are displayed first.
   */
  private final PriorityQueue<RGBMessage> drainedMessageQueue = new PriorityQueue<>();

  /** Creates a new RGBSubsystem. */
  public RGBSubsystem() {
    candle = new CANdle(Lights.CANDLE_ID);
    // turn off the LEDs when the can chain fails
    candle.configLOSBehavior(true);
    candle.configLEDType(LEDStripType.GRB);
  }

  /**
   * Shows a message on the LEDs. Non blocking and thread safe. Caller is responsible for expiring
   * the message when it is no longer needed. Priority determines the order in which messages are
   * displayed. Equal priority message order is undefined. Message will not be processed until the
   * next periodic if you are not on the main thread.
   *
   * @param color The color to display
   * @param pattern The pattern to display it with
   * @param priority The priority of the message
   * @return The message object that was created, which can be used to expire the message
   */
  public RGBMessage showMessage(RGBColor color, PatternTypes pattern, MessagePriority priority) {
    RGBMessage message = new RGBMessage(color, pattern, priority);
    threadSafeMessageSink.offer(message);
    return message;
  }

  private void showPulseColor(RGBColor color) {
    candle.animate(new SingleFadeAnimation(color.r, color.g, color.b, 0, .7, Lights.NUM_LEDS));
    lastAppliedAnimation = Optional.of(CurrentAnimationTypes.SINGLE_FADE);
    lastAppliedColor = Optional.of(color);
  }

  private void showBounceColor(RGBColor color) {
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
    lastAppliedAnimation = Optional.of(CurrentAnimationTypes.LARSON);
    lastAppliedColor = Optional.of(color);
  }

  private void showStrobeColor(RGBColor color) {
    candle.animate(new StrobeAnimation(color.r, color.g, color.b, 0, .2, Lights.NUM_LEDS));
    lastAppliedAnimation = Optional.of(CurrentAnimationTypes.STROBE);
    lastAppliedColor = Optional.of(color);
  }

  private void showMessage(RGBMessage message) {
    switch (message.pattern) {
      case PULSE -> showPulseColor(message.color);
      case BOUNCE -> showBounceColor(message.color);
      case STROBE -> showStrobeColor(message.color);
    }
  }

  private void showRainbow() {
    candle.animate(new RainbowAnimation(.2, .5, Lights.NUM_LEDS));
    lastAppliedAnimation = Optional.of(CurrentAnimationTypes.RAINBOW);
    lastAppliedColor = Optional.empty();
  }

  @Override
  public void periodic() {
    { // drain the queue
      RGBMessage message;
      while ((message = threadSafeMessageSink.poll()) != null) {
        drainedMessageQueue.add(message);
      }
    }

    boolean isMessageDisplayed = false;
    while (!drainedMessageQueue.isEmpty() && !isMessageDisplayed) {
      RGBMessage message = drainedMessageQueue.peek();
      if (message.isExpired) {
        drainedMessageQueue.remove();
        continue;
      }

      // if the message is the same as the last message, don't display it again
      if (!(lastAppliedColor.isPresent()
          && lastAppliedColor.get().equals(message.color)
          && lastAppliedAnimation.isPresent()
          && lastAppliedAnimation.get() == message.pattern.type)) {
        showMessage(message);
      }

      isMessageDisplayed = true;
    }

    if (!isMessageDisplayed
        && !(lastAppliedAnimation.isPresent()
            && lastAppliedAnimation.get() == CurrentAnimationTypes.RAINBOW)) {
      showRainbow();
    }
  }
}
