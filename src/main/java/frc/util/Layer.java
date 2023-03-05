package frc.util;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/**
 * An object to use a {@link Trigger} or {@link Trigger} as a layerSwitch to form two layers of
 * virtual Triggers, with the switch on or off
 */
public class Layer {
  /**
   * the trigger that is used for on and off states for the layer such that the states correspond to
   * the state of the trigger
   */
  private final Trigger layerSwitch;

  private final EventLoop eventLoop;

  /**
   * Constructs a new Layer based off of an arbitrary boolean supplier.
   *
   * @param layerSwitch BooleanSupplier representing the layer root.
   */
  public Layer(BooleanSupplier layerSwitch) {
    this(new Trigger(layerSwitch));
  }

  /**
   * Constructs a new Layer based off of a {@link Trigger}.
   *
   * @param layerSwitch WPILib Trigger representing the layer root.
   */
  public Layer(Trigger layerSwitch) {
    this(CommandScheduler.getInstance().getDefaultButtonLoop(), layerSwitch);
  }

  public Layer(EventLoop eventLoop, Trigger layerSwitch) {
    this.eventLoop = eventLoop;
    this.layerSwitch = layerSwitch;
  }

  /**
   * gets the Trigger that controls the layering
   *
   * @return
   */
  public Trigger getLayerSwitch() {
    return layerSwitch;
  }

  public void whenChanged(BooleanConsumer method) {
    Trigger layerTrigger = new Trigger(eventLoop, this.layerSwitch);
    layerTrigger.onTrue(
        new InstantCommand(
            () -> {
              method.accept(true);
            }));
    layerTrigger.onFalse(
        new InstantCommand(
            () -> {
              method.accept(false);
            }));
  }

  private static class BooleanRef {
    boolean val;

    public BooleanRef(boolean val) {
      this.val = val;
    }
  }

  /**
   * Combines a {@link Trigger} with the layer switch to form a virtual trigger that is only true
   * when the layer switch is true and the trigger is true.
   *
   * @param trigger the {@link Trigger} that is used in tandem with the layer for the virtual
   *     Trigger
   * @return virtual {@link Trigger} that is pressed when the layer switch and the provided Trigger
   *     is pressed
   */
  public Trigger on(Trigger trigger) {
    return new Trigger(eventLoop, layerSwitch.and(trigger));
  }

  /**
   * Combines a {@link BooleanSupplier} with the layer switch to form a virtual trigger that is only
   * true when the layer switch is true and the trigger is true.
   *
   * @param Trigger the {@link BooleanSupplier} that is used in tandem with the layer for the
   *     virtual Trigger
   * @return virtual {@link Trigger} that is pressed when the layer switch and the provided Trigger
   *     is pressed
   */
  public Trigger on(BooleanSupplier boolSupplier) {
    return on(new Trigger(eventLoop, boolSupplier));
  }

  /**
   * Combines a {@link Trigger} with the layer switch to form a virtual trigger that is only true
   * when the layer switch is false and the trigger is true. Additionally, the virtual trigger will
   * be false if the layer switch was released but the trigger was still pressed, until the trigger
   * is released.
   *
   * @param trigger the {@link Trigger} that is used in tandem with the layer for the virtual
   *     Trigger
   * @return virtual {@link Trigger} that is pressed when the layer switch is not pressed, but the
   *     provided Trigger is pressed
   */
  public Trigger off(Trigger trigger) {
    final BooleanRef layerSwitchWasTrue = new BooleanRef(false);
    return new Trigger(
        eventLoop,
        () -> {
          final boolean layerSwitchVal = layerSwitch.getAsBoolean();
          final boolean triggerVal = trigger.getAsBoolean();

          if (layerSwitchVal) {
            layerSwitchWasTrue.val = true;
            return false;
          }

          if (!layerSwitchVal && !triggerVal) {
            layerSwitchWasTrue.val = false;
            return false;
          }

          if (layerSwitchWasTrue.val) {
            return false;
          }

          return !layerSwitchVal && triggerVal;
        });
  }

  /**
   * Combines a {@link BooleanSupplier} with the layer switch to form a virtual trigger that is only
   * true when the layer switch is false and the trigger is true. Additionally, the virtual trigger
   * will be false if the layer switch was released but the trigger was still pressed, until the
   * trigger is released.
   *
   * @param Trigger the {@link BooleanSupplier} that is used in tandem with the layer for the
   *     virtual Trigger
   * @return virtual {@link Trigger} that is pressed when the layer switch is not pressed, but the
   *     provided Trigger is pressed
   */
  public Trigger off(BooleanSupplier boolSupplier) {
    return off(new Trigger(eventLoop, boolSupplier));
  }
}
