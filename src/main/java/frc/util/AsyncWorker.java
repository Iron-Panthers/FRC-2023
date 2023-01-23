package frc.util;

import java.util.Optional;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Consumer;

public class AsyncWorker {
  private final ExecutorService executor = Executors.newSingleThreadExecutor();

  AsyncWorker() {}

  public class Result<T> {
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

    /**
     * Register a callback to be called when the result is ready, from the worker thread. Even if
     * the result is already, the callback will still be submitted to the worker thread for
     * consistency.
     *
     * <p>Be very careful with this method, as it can deadlock the worker thread. Only call it if
     * you are sure it is the correct solution to your problem. You can probably just use {@link
     * #get()} in your periodic instead.
     *
     * @param callback The callback to be called when the result is ready.
     */
    public void subscribe(Consumer<T> callback) {
      executor.submit(
          () -> {
            try {
              callback.accept(future.get());
            } catch (InterruptedException e) {
              Thread.currentThread().interrupt();
            } catch (ExecutionException e) {
              e.printStackTrace();
            }
          });
    }

    /**
     * Register a callback to be run when the result is ready, from the main thread. Even if the
     * result is already, the callback will still be submitted to the main thread for consistency.
     *
     * <p>Be very careful with this method, as it can deadlock the worker thread. Only call it if
     * you are sure it is the correct solution to your problem. You can probably just use {@link
     * #get()} in your periodic instead.
     *
     * @param callback The callback to be run when the result is ready.
     */
    public void subscribe(Runnable callback) {
      this.subscribe(value -> callback.run());
    }
  }

  public <T> Result<T> submit(Callable<T> callable) {
    return new Result<>(executor.submit(callable));
  }
}
