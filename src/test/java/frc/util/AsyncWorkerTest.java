package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.ConfigSystemErrorLogging;
import frc.UtilTest;
import java.util.Optional;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;

public class AsyncWorkerTest {

  // before each test, hide the error logging
  @BeforeAll
  public static void setUp() {
    ConfigSystemErrorLogging.hide();
  }

  @AfterAll
  public static void tearDown() {
    ConfigSystemErrorLogging.show();
  }

  /**
   * This function is dangerous and blocking. Don't do this outside of unit tests. If the task does
   * not complete, this will lock up the test thread for 10 ms.
   *
   * @param <T> The type of the result
   * @param result The result to wait for
   */
  public static <T> void await(AsyncWorker.Result<T> result) {
    CountDownLatch latch = new CountDownLatch(1);
    new Thread(
            () -> {
              while (true) {
                boolean wasHidden = ConfigSystemErrorLogging.isHidden();
                ConfigSystemErrorLogging.hide();
                boolean finished = !result.get().isPresent();
                if (!wasHidden) {
                  ConfigSystemErrorLogging.show();
                }
                if (finished) {
                  latch.countDown();
                  break;
                }
              }
            })
        .start();
    try {
      latch.await(10, TimeUnit.MILLISECONDS);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  @UtilTest
  public void asyncWorkerResolves() {
    AsyncWorker worker = new AsyncWorker();

    CountDownLatch delay = new CountDownLatch(1);

    AsyncWorker.Result<Integer> result =
        worker.submit(
            () -> {
              delay.await();
              return 1;
            });

    assertEquals(Optional.empty(), result.get(), "Result should not be ready yet");

    delay.countDown();

    await(worker, result);

    assertEquals(Optional.of(1), result.get(), "result should be 1 after worker finishes");
  }

  @UtilTest
  public void asyncWorkerFailsToResolve() throws RuntimeException {
    AsyncWorker worker = new AsyncWorker();

    CountDownLatch delay = new CountDownLatch(1);

    AsyncWorker.Result<Integer> result =
        worker.submit(
            () -> {
              delay.await();
              throw new RuntimeException("Test exception");
            });

    assertEquals(Optional.empty(), result.get(), "Result should not be ready yet");

    delay.countDown();

    await(worker, result);

    assertEquals(Optional.empty(), result.get(), "result should be empty after worker fails");
  }

  @UtilTest
  public void asyncWorkerResolvesMultipleTasks() {
    AsyncWorker worker = new AsyncWorker();

    CountDownLatch delay = new CountDownLatch(1);

    AsyncWorker.Result<Integer> result1 =
        worker.submit(
            () -> {
              delay.await();
              return 1;
            });

    AsyncWorker.Result<Integer> result2 =
        worker.submit(
            () -> {
              delay.await();
              throw new RuntimeException("Test exception");
            });

    AsyncWorker.Result<Integer> result3 =
        worker.submit(
            () -> {
              delay.await();
              return 3;
            });

    assertEquals(Optional.empty(), result1.get(), "Result should not be ready yet");
    assertEquals(Optional.empty(), result2.get(), "Result should not be ready yet");
    assertEquals(Optional.empty(), result3.get(), "Result should not be ready yet");

    delay.countDown();

    await(worker, result1);
    await(worker, result2);
    await(worker, result3);

    assertEquals(Optional.of(1), result1.get(), "result should be 1 after worker finishes");
    assertEquals(Optional.empty(), result2.get(), "result should be empty after worker fails");
    assertEquals(Optional.of(3), result3.get(), "result should be 3 after worker finishes");
  }
}
