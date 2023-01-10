package frc.util;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/**
 * An object to use a {@link Button} or {@link Trigger} as a layerSwitch to form two layers of
 * virtual buttons, with the switch on or off
 */
public class Layer {
  /**
   * the trigger that is used for on and off states for the layer such that the states correspond to
   * the state of the trigger
   */
  private final Trigger layerSwitch;

  /**
   * Constructs a new Layer based off of an arbitrary boolean supplier.
   *
   * @param layerSwitch BooleanSupplier representing the layer root.
   */
  public Layer(BooleanSupplier layerSwitch) {
    this.layerSwitch = new Button(layerSwitch);
  }

  /**
   * Constructs a new Layer based off of a {@link Trigger}.
   *
   * @param layerSwitch WPILib Trigger representing the layer root.
   */
  public Layer(Trigger layerSwitch) {
    this.layerSwitch = layerSwitch;
  }

  /**
   * gets the button that controls the layering
   *
   * @return
   */
  public Trigger getLayerSwitch() {
    return layerSwitch;
  }

  public void whenChanged(BooleanConsumer method) {
    Button layerButton = new Button(this.layerSwitch);
    layerButton.whenPressed(
        () -> {
          method.accept(true);
        });
    layerButton.whenReleased(
        () -> {
          method.accept(false);
        });
  }

  /**
   * Combines a {@link Trigger} with the layer root provided that the layer switch is activated.
   *
   * @param button the {@link Trigger} that is used in tandem with the layer for the virtual button
   * @return virtual {@link Button} that is pressed when the layer switch and the provided button is
   *     pressed
   */
  public Button on(Trigger button) {
    return new Button(layerSwitch.and(button));
  }

  /**
   * Combines a {@link BooleanSupplier} with the layer root provided that the layer switch is
   * activated.
   *
   * @param button the {@link BooleanSupplier} that is used in tandem with the layer for the virtual
   *     button
   * @return virtual {@link Button} that is pressed when the layer switch and the provided button is
   *     pressed
   */
  public Button on(BooleanSupplier boolSupplier) {
    return on(new Button(boolSupplier));
  }

  /**
   * Combines a {@link Trigger} with the layer root provided that the layer switch is not activated.
   *
   * @param button the {@link Trigger} that is used in tandem with the layer for the virtual button
   * @return virtual {@link Button} that is pressed when the layer switch is not pressed, but the
   *     provided button is pressed
   */
  public Button off(Trigger button) {
    return new Button(layerSwitch.negate().and(button));
  }

  /**
   * Combines a {@link BooleanSupplier} with the layer root provided that the layer switch is not
   * activated.
   *
   * @param button the {@link BooleanSupplier} that is used in tandem with the layer for the virtual
   *     button
   * @return virtual {@link Button} that is pressed when the layer switch is not pressed, but the
   *     provided button is pressed
   */
  public Button off(BooleanSupplier boolSupplier) {
    return off(new Button(boolSupplier));
  }
}
