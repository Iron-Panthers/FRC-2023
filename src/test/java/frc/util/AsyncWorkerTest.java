package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.UtilTest;
import java.util.Optional;
import java.util.concurrent.CountDownLatch;

public class AsyncWorkerTest {

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
}
