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
}
