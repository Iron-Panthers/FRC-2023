package frc;

public class ConfigSystemErrorLogging {
  private static final java.io.PrintStream realErrorStream = System.err;
  private static final java.io.PrintStream fakeErrorStream =
      new java.io.PrintStream(
          new java.io.OutputStream() {
            public void write(int i) {}
          });

  public static void hide() {
    System.setErr(fakeErrorStream);
  }

  public static void show() {
    System.setErr(realErrorStream);
  }
}
