package frc.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.UtilTest;
import org.junit.jupiter.api.BeforeEach;

public class LayerTests {
  private boolean switchVal = false;
  private Trigger layerSwitch = new Trigger(() -> switchVal);

  private boolean buttonAVal = false;
  private Trigger buttonA = new Trigger(() -> buttonAVal);

  private boolean buttonBVal = false;
  private Trigger buttonB = new Trigger(() -> buttonBVal);

  private Layer layer = new Layer(layerSwitch);

  private Trigger buttonALayerOff = layer.off(buttonA);
  private Trigger buttonALayerOn = layer.on(buttonA);

  private Trigger buttonBLayerOff = layer.off(buttonB);
  private Trigger buttonBLayerOn = layer.on(buttonB);

  @BeforeEach
  public void setup() {
    switchVal = false;
    buttonAVal = false;
    buttonBVal = false;
  }

  @UtilTest
  public void buttonTrueWhenLayerOffAndButtonOn() {
    buttonAVal = true;
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
    assertFalse(buttonALayerOff);
    assertFalse(buttonBLayerOn);
    switchVal = true;
    assertTrue(buttonBLayerOn);
  }
}
