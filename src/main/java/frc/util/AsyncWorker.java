package frc.util;

import java.util.Optional;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class AsyncWorker {
  private final ExecutorService executor = Executors.newSingleThreadExecutor();

  AsyncWorker() {}

  public static class Result<T> {
    private final Future<T> future;
    private Optional<T> optionalValue = Optional.empty();

    Result(Future<T> future) {
      this.future = future;
    }

    public Optional<T> get() {
      if (!optionalValue.isPresent() && future.isDone()) {
        try {
          optionalValue = Optional.of(future.get());
          return optionalValue;
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        } catch (ExecutionException e) {
          e.printStackTrace();
        }
      }
      return optionalValue;
    }
  }

  public <T> Result<T> submit(Callable<T> callable) {
    return new Result<>(executor.submit(callable));
  }
}
