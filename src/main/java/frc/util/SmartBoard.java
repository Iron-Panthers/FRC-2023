package frc.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class SmartBoard {

  public record Range(double min, double max) {
    private Map<String, Object> asConfig() {
      return Map.of("min", min, "max", max);
    }
  }

  private GenericEntry entry;
  private DoubleSupplier supplier;
  private DoubleConsumer consumer;

  public SmartBoard(
      ShuffleboardTab tab,
      String name,
      DoubleSupplier supplier,
      DoubleConsumer consumer,
      Range range) {
    this.supplier = supplier;
    this.consumer = consumer;
    entry =
        tab.add(name, supplier.getAsDouble())
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(range.asConfig())
            .getEntry();
  }

  public void poll() {
    consumer.accept(entry.getDouble(supplier.getAsDouble()));
  }
}
