package frc.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.UtilTest;
import org.junit.jupiter.api.BeforeEach;

public class LayerTests {
  private boolean switchVal = false;
  private Button layerSwitch = new Button(() -> switchVal);

  private boolean buttonAVal = false;
  private Button buttonA = new Button(() -> buttonAVal);

  private boolean buttonBVal = false;
  private Button buttonB = new Button(() -> buttonBVal);

  private Layer layer = new Layer(layerSwitch);

  private Button buttonALayerOff = layer.off(buttonA);
  private Button buttonALayerOn = layer.on(buttonA);

  private Button buttonBLayerOff = layer.off(buttonB);
  private Button buttonBLayerOn = layer.on(buttonB);

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
