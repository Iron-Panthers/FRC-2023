package frc.util;

import java.util.function.BooleanSupplier;

public class ControllerUtil {
  /**
   * Scales radial deadband
   *
   * <p>The deadband value is set to be the new zero
   *
   * @param value the raw value
   * @param deadband the deadband range
   * @return the value with radial deadband applied
   */
  public static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private ControllerUtil() {}
  /**
   * Compares a layer button with a layer state, and if they match returns the layered button
   *
   * @param layer the boolean supplier that is the layer switch
   * @param layerState the state of the layer switch that is valid
   * @param button the button inside the layer
   * @return true if the layer is enabled and the button is pressed
   */
  public static BooleanSupplier cumBooleanSupplier(
      BooleanSupplier layer, boolean layerState, BooleanSupplier button) {
    return () -> layer.getAsBoolean() == layerState && button.getAsBoolean();
  }
}
