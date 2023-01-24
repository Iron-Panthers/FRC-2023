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

    /**
     * Get the result of the computation. If the computation is not complete, this will return an
     * empty optional. Optional values are cached for subsequent calls. This method will not block.
     *
     * @return the value of the result if it is ready, or empty if it is not ready. If the task
     *     throws an error, empty will always be returned.
     */
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
     * the result is already finished, the callback will still be submitted to the worker thread for
     * consistency.
     *
     * <p>Be very careful with this method, as it can deadlock the worker thread. Only call it if
     * you are sure it is the correct solution to your problem. You can probably just use {@link
     * #get()} in your periodic instead.
     *
     * @param callback The callback to be called when the result is ready.
     */
    public void subscribe(Consumer<Optional<T>> callback) {
      executor.submit(
          () -> {
            try {
              // Wait for the result to be ready
              future.get();
            } catch (InterruptedException e) {
              Thread.currentThread().interrupt();
            } catch (ExecutionException e) {
              e.printStackTrace();
            }
            callback.accept(this.get());
          });
    }

    /**
     * Register a callback to be run when the result is ready, from the worker thread. Even if the
     * result is already finished, the callback will still be submitted to the main thread for
     * consistency.
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

  /**
   * Submit a task to be run asynchronously. The task will be run on a single worker thread owned by
   * the AsyncWorker. If the task blocks, it will block the worker thread, so be careful. Errors in
   * the task will be printed to the console. Tasks will be run in the order they are submitted.
   *
   * @param <T> The type of the result of the task.
   * @param callable The task to be run.
   * @return A Result object that can be used to check on the {@link Optional} result of the task.
   */
  public <T> Result<T> submit(Callable<T> callable) {
    return new Result<>(executor.submit(callable));
  }
}
