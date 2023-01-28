package frc.util;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Consumer;

public class AsyncWorker {
  private final ExecutorService executor = Executors.newSingleThreadExecutor();
  private final ArrayList<Subscription<?>> subscriptions = new ArrayList<>();
  private final ArrayList<Result<?>> createdResults = new ArrayList<>();

  private class Subscription<T> {
    public final Result<T> result;
    public final Consumer<Optional<T>> callback;

    Subscription(Result<T> result, Consumer<Optional<T>> callback) {
      this.result = result;
      this.callback = callback;
    }

    boolean evaluate() {
      var optionalValue = result.get();
      if (optionalValue.isPresent()) {
        callback.accept(result.get());
      }
      return optionalValue.isPresent();
    }
  }

  public AsyncWorker() {}

  public class Result<T> {
    private final Future<T> future;
    private Optional<Error> error = Optional.empty();
    private Optional<T> optionalValue = Optional.empty();

    Result(Future<T> future) {
      this.future = future;
      createdResults.add(this);
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
          error = Optional.of(new Error(e.getCause()));
          e.printStackTrace();
        }
      }
      return optionalValue;
    }

    /**
     * Get the error of the computation. If the computation is not complete, or if the computation
     * did not throw an error, this will return an empty optional. Optional values are cached for
     * subsequent calls. This method will not block.
     *
     * @return the error of the result if it is ready, or empty if it is not ready. If the task
     *     completes successfully, empty will always be returned.
     */
    public Optional<Error> getError() {
      return error;
    }

    /**
     * Determine if the computation is complete. This method will not block. Equivalent to checking
     * if either the error or result is present. If you never call {@link #get()}, this will never
     * be true.
     *
     * @return true if the computation is complete, false otherwise.
     */
    public boolean hasResolved() {
      return get().isPresent() || getError().isPresent();
    }

    /**
     * Register a callback to be called when the result is ready, from the context the heartbeat
     * function is called. Because the heartbeat must be called to evaluate subscriptions, they will
     * not resolve when your command becomes unscheduled.is notis not done done
     *
     * <p>This may not be the right solution to your problem. If you just want to get the value when
     * it is done, you can probably just use {@link #get()} in your periodic instead. You will
     * probably regret modifying state or controlling subsystems from a subscription.
     *
     * @param callback The callback to be called when the result is ready.
     */
    public void subscribe(Consumer<Optional<T>> callback) {
      subscriptions.add(new Subscription<>(this, callback));
    }

    /**
     * Register a callback to be run when the result is ready, from the context the heartbeat
     * function is run. Because the heartbeat must be called to evaluate subscriptions, they will
     * not resolve when your command becomes unscheduled.
     *
     * <p>This may not be the right solution to your problem. If you just want to do something when
     * the value changes, you can probably just use {@link #get()} in your periodic instead. You
     * will probably regret modifying state or controlling subsystems from a subscription.
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

  /**
   * Evaluate all subscriptions. This should be called periodically from the context of that your
   * {@link AsyncWorker} exists in. This will call all callbacks for subscriptions that have been
   * resolved. You should not pass this function elsewhere, because your subscriptions should not
   * resolve when your command is not scheduled.
   *
   * <p>For example, spawning a thread to call the heartbeat function is a very bad idea, because
   * your command state can be mutated when it is not running, or your subsystems can be written to
   * when you do not hold the mutex. Do not do that.
   */
  public void heartbeat() {
    subscriptions.removeIf(Subscription::evaluate);
  }

  /**
   * Get the number of subscriptions that have not been resolved. This is useful for testing. You
   * probably shouldn't do anything else with this value...
   *
   * @return The number of subscriptions that have not been resolved.
   */
  public int getPendingSubscriptions() {
    return subscriptions.size();
  }

  public void purge() {
    subscriptions.clear();
    for (Result<?> result : createdResults) {
      result.future.cancel(true);
    }
    createdResults.clear();
  }
}
