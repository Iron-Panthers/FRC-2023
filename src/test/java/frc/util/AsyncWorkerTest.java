package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.UtilTest;
import java.util.Optional;
import java.util.concurrent.CountDownLatch;

public class AsyncWorkerTest {

  /**
   * This function is dangerous and blocking. Don't do this outside of unit tests. If the task does
   * not complete, this will deadlock the main thread.
   *
   * @param <T> The type of the result
   * @param result The result to wait for
   */
  public static <T> void await(AsyncWorker.Result<T> result) {
    CountDownLatch latch = new CountDownLatch(1);
    result.subscribe(
        () -> {
          latch.countDown();
        });
    try {
      latch.await();
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

    await(result);

    assertEquals(Optional.of(1), result.get(), "result should be 1 after worker finishes");
  }

  @UtilTest
  public void asyncWorkerFailsToResolve() {
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

    await(result);

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

    await(result1);
    await(result2);
    await(result3);

    assertEquals(Optional.of(1), result1.get(), "result should be 1 after worker finishes");
    assertEquals(Optional.empty(), result2.get(), "result should be empty after worker fails");
    assertEquals(Optional.of(3), result3.get(), "result should be 3 after worker finishes");
  }
}
