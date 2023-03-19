package frc.util;

import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.Mockito.mockConstruction;
import static org.mockito.Mockito.verify;

import edu.wpi.first.wpilibj.Filesystem;
import frc.UtilTest;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import org.mockito.MockedConstruction;

public class CSVTests {
  @UtilTest
  public void csvSerializesCorrectly() {

    record Row(String a, Integer b, Double c) {}

    // we need to mock FileWriter
    try (MockedConstruction<FileWriter> mocked = mockConstruction(FileWriter.class)) {
      CSV<Row> csv =
          new CSV<>(
              Filesystem.getDeployDirectory().toPath().resolve("test.csv"),
              List.of(CSV.column("a", Row::a), CSV.column("b", Row::b), CSV.column("c", Row::c)));

      csv.write(new Row("word", 1, 2.0));

      // we need to check that the file was written to correctly
      try {
        verify(mocked.constructed().get(0))
            .write("a,b,c" + System.lineSeparator() + "word,1,2.0" + System.lineSeparator());
      } catch (IOException e) {
        fail(e);
      }
    }
  }
}
