package frc.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.StringWriter;
import java.nio.file.Path;
import java.util.Map;
import java.util.function.Function;

public class CSV<E> {

  private final Map<String, Function<E, ?>> rowMap;
  private final File file;
  private final String header;

  public CSV(Path path, Map<String, Function<E, ?>> rowMap) {
    this.file = path.toFile();
    this.rowMap = rowMap;
    this.header =
        rowMap.keySet().stream()
            .reduce((a, b) -> a + "," + b)
            .orElseThrow(IllegalStateException::new);
  }

  public void write(E row) {
    try (var writer = new StringWriter()) {
      if (!file.isFile()) {
        writer.write(header);
        writer.write(System.lineSeparator());
      }
      writer.write(
          rowMap.values().stream()
              .map(f -> f.apply(row))
              .map(Object::toString)
              .reduce((a, b) -> a + "," + b)
              .orElseThrow(IllegalStateException::new));
      writer.write(System.lineSeparator());
      try (var fileWriter = new FileWriter(file, true)) {
        fileWriter.write(writer.toString());
      }
    } catch (IOException e) {
      System.err.println("Failed to write to file.");
      e.printStackTrace();
    }
  }
}
