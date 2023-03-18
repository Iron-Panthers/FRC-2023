package frc.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.UtilTest;
import org.junit.jupiter.api.BeforeEach;

public class LayerTests {
  private EventLoop eventLoop = new EventLoop();

  private boolean switchVal = false;
  private Trigger layerSwitch = new Trigger(eventLoop, () -> switchVal);

  private boolean buttonAVal = false;
  private Trigger buttonA = new Trigger(eventLoop, () -> buttonAVal);

  private boolean buttonBVal = false;
  private Trigger buttonB = new Trigger(eventLoop, () -> buttonBVal);

  private Layer layer = new Layer(eventLoop, layerSwitch);

  private Trigger buttonALayerOff = layer.off(buttonA);
  private Trigger buttonALayerOn = layer.on(buttonA);

  private Trigger buttonBLayerOff = layer.off(buttonB);
  private Trigger buttonBLayerOn = layer.on(buttonB);

  @BeforeEach
  public void setup() {
    switchVal = false;
    buttonAVal = false;
    buttonBVal = false;

    // if you never bind to a trigger, it won't fire
    // triggers are lazy
    // our triggers have state, so we need to bind them to something to test them
    buttonALayerOff.onTrue(new InstantCommand());
    buttonALayerOn.onTrue(new InstantCommand());
    buttonBLayerOff.onTrue(new InstantCommand());
    buttonBLayerOn.onTrue(new InstantCommand());

    // we need to run the event loop once to initialize the triggers
    eventLoop.poll();
  }

  @UtilTest
  public void buttonTrueWhenLayerOffAndButtonOn() {
    buttonAVal = true;
    eventLoop.poll();
    assertTrue(buttonALayerOff);
  }

  @UtilTest
  public void buttonFalseWhenFalseRegardlessOfLayer() {
    assertFalse(buttonALayerOff);
    assertFalse(buttonALayerOn);
  }

  @UtilTest
  public void buttonTrueConditionalOnLayer() {
    buttonBVal = true;
    eventLoop.poll();
    assertFalse(buttonALayerOff);
    assertFalse(buttonBLayerOn);
    switchVal = true;
    eventLoop.poll();
    assertTrue(buttonBLayerOn);
  }

  @UtilTest
  public void buttonBecomesTrueWhenLayerAndButtonTurnsOn() {
    assertFalse(buttonALayerOff);
    assertFalse(buttonALayerOn);
    switchVal = true;
    eventLoop.poll();
    assertFalse(buttonALayerOff);
    assertFalse(buttonALayerOn);
    buttonAVal = true;
    eventLoop.poll();
    assertFalse(buttonALayerOff);
    assertTrue(buttonALayerOn);
  }

  @UtilTest
  public void releasingLayerSwitchDoesNotTriggerLayerOffButtonAndCancelsLayerOn() {
    switchVal = true;
    eventLoop.poll();
    buttonAVal = true;
    eventLoop.poll();
    assertTrue(
        buttonALayerOn,
        "buttonALayerOn should be true because the layer is on and the button is pressed");
    switchVal = false;
    eventLoop.poll();
    assertFalse(buttonALayerOn, "buttonALayerOn should be false because the layer is off");
    assertFalse(
        buttonALayerOff,
        "buttonALayerOff should be false even though the button is pressed, because the button and layer haven't been released");
  }

  @UtilTest
  public void pressingLayerSwitchAndButtonAtSameTimeTriggersLayerOnButton() {
    buttonAVal = true;
    switchVal = true;
    eventLoop.poll();
    assertTrue(
        buttonALayerOn,
        "buttonALayerOn should be true because the layer is on and the button is pressed");
  }

  @UtilTest
  public void releasingLayerSwitchAndButtonAllowsOffLayerToBePressedAgain() {
    switchVal = true;
    eventLoop.poll();
    buttonAVal = true;
    eventLoop.poll();
    switchVal = false;
    eventLoop.poll();
    assertFalse(buttonALayerOn, "buttonALayerOn should be false because the layer is off");
    assertFalse(
        buttonALayerOff,
        "buttonALayerOff should be false even though the button is pressed, because the button and layer haven't been released");
    buttonAVal = false;
    eventLoop.poll();
    assertFalse(
        buttonALayerOff, "buttonALayerOff should be false because the button isn't pressed");

    buttonAVal = true;
    eventLoop.poll();
    assertTrue(
        buttonALayerOff,
        "buttonALayerOff should be true because the button is pressed and the layer is off");
  }

  @UtilTest
  public void layerOffRemainsOffForAsLongAsButtonIsNotReleasedAfterLayerOn() {
    switchVal = true;
    buttonAVal = true;
    eventLoop.poll();
    switchVal = false;
    eventLoop.poll();
    assertFalse(
        buttonALayerOff,
        "buttonALayerOff should be false because the button hasn't been released since layer usage");
    eventLoop.poll();
    assertFalse(
        buttonALayerOff,
        "buttonALayerOff should be false because the button hasn't been released since layer usage");
    eventLoop.poll();
    assertFalse(
        buttonALayerOff,
        "buttonALayerOff should be false because the button hasn't been released since layer usage");
    buttonAVal = false;
    eventLoop.poll();
    assertFalse(buttonALayerOff, "buttonALayerOff should be false because the button is released");
    buttonAVal = true;
    eventLoop.poll();
    assertTrue(
        buttonALayerOff,
        "buttonALayerOff should be true because the button is pressed and the layer is off");
  }

  @UtilTest
  public void pressingLayerOffButtonRepeatedlyWorks() {
    for (int i = 0; i < 3; i++) {
      buttonAVal = true;
      eventLoop.poll();
      assertTrue(buttonALayerOff, "buttonALayerOff should be true because the button is pressed");
      assertFalse(buttonALayerOn, "buttonALayerOn should be false because the layer is off");
      buttonAVal = false;
      eventLoop.poll();
      assertFalse(
          buttonALayerOff, "buttonALayerOff should be false because the button is released");
      assertFalse(
          buttonALayerOn,
          "buttonALayerOn should be false because the layer is off and button is released");
    }
  }

  @UtilTest
  public void pressingLayerOnButtonRepeatedlyWorks() {
    switchVal = true;
    eventLoop.poll();
    for (int i = 0; i < 3; i++) {
      buttonAVal = true;
      eventLoop.poll();
      assertFalse(buttonALayerOff, "buttonALayerOff should be false because the layer is on");
      assertTrue(buttonALayerOn, "buttonALayerOn should be true because the button is pressed");
      buttonAVal = false;
      eventLoop.poll();
      assertFalse(
          buttonALayerOff,
          "buttonALayerOff should be false because the layer is on and button is released");
      assertFalse(
          buttonALayerOn,
          "buttonALayerOn should be false because the layer is on and button is released");
    }
  }

  @UtilTest
  public void
      releasingAndRepressingLayerSwitchWhileHoldingTriggerReTriggersLayerOnButDoesNotTriggerLayerOff() {
    buttonAVal = true;
    eventLoop.poll();
    assertTrue(buttonALayerOff, "buttonALayerOff should be true because the button is pressed");
    assertFalse(buttonALayerOn, "buttonALayerOn should be false because the layer is off");
    for (int i = 0; i < 3; i++) {
      switchVal = true;
      eventLoop.poll();
      assertFalse(buttonALayerOff, "buttonALayerOff should be false because the layer is on");
      assertTrue(buttonALayerOn, "buttonALayerOn should be true because the button is pressed");
      switchVal = false;
      eventLoop.poll();
      assertFalse(buttonALayerOn, "buttonALayerOn should be false because the layer is off");
      assertFalse(
          buttonALayerOff,
          "buttonALayerOff should be false because the button is still pressed and the layer is off");
    }
  }
}
