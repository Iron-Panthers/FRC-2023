package frc.util;

import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.mockConstruction;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import edu.wpi.first.wpilibj.Filesystem;
import frc.UtilTest;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
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

  @UtilTest
  public void longerCSVSerializesCorrectly() {

    record Row(String a, Integer b, Double c) {}

    // we need to mock FileWriter
    try (MockedConstruction<FileWriter> mocked = mockConstruction(FileWriter.class)) {
      Path path = mock(Path.class);
      // we need to return a mock file when we call toFile()
      File file = mock(File.class);
      when(path.toFile()).thenReturn(file);
      boolean[] isFile = {false};
      when(file.isFile()).thenAnswer(i -> isFile[0]);

      CSV<Row> csv =
          new CSV<>(
              path,
              List.of(CSV.column("a", Row::a), CSV.column("b", Row::b), CSV.column("c", Row::c)));

      csv.write(new Row("word", 1, 2.0));
      isFile[0] = true;
      csv.write(new Row("word", 17, 3.5));
      csv.write(new Row("other word", 12, Double.MIN_VALUE));

      // we need to check that the file was written to correctly
      List<FileWriter> writers = mocked.constructed();
      List<String> expected =
          List.of(
              "a,b,c" + System.lineSeparator() + "word,1,2.0" + System.lineSeparator(),
              "word,17,3.5" + System.lineSeparator(),
              "other word,12,4.9E-324" + System.lineSeparator());

      for (int i = 0; i < writers.size(); i++) {
        try {
          verify(writers.get(i)).write(expected.get(i));
        } catch (IOException e) {
          fail(e);
        }
      }
    }
  }
}
