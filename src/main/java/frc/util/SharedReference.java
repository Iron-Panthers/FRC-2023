package frc.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

/** A class to facilitate a shared reference to data. */
public class SharedReference<T> {
  /**
   * A class that stores a function to call when data is updated. It only exposes a method to
   * destroy the subscription, preventing it from being called more. Subscriptions are called as
   * soon as the value is updated.
   */
  public class Subscription {
    private final Function<T, Boolean> function;
    private boolean destroyed = false;

    private Subscription(Function<T, Boolean> function) {
      this.function = function;
      subscriptions.add(this);
    }

    private Subscription(Consumer<T> consumer) {
      this(
          (T currentData) -> {
            consumer.accept(currentData);
            return true;
          });
    }

    /**
     * Call the consumer with the current value.
     *
     * @param data the data to pass to the consumer
     * @return true if the subscription is still valid, false if it has been destroyed
     */
    private boolean update(T data) {
      if (destroyed) return false;
      return function.apply(data);
    }

    /** Destroy the subscription. A destroyed subscription will not run again. */
    public void destroy() {
      destroyed = true;
    }
  }

  private T data;
  private List<Subscription> subscriptions = new ArrayList<>();

  public SharedReference(T data) {
    this.data = data;
  }

  /**
   * Get the data. Data will never be null. You shouldn't store the data--this call is free, and
   * storing it would prevent reactivity. Only store it if subsequent operations wouldn't make sense
   * if the data changed.
   *
   * @return the data. Copies are not made, so the reference should not be modified.
   */
  public T get() {
    return data;
  }

  /**
   * Set the data. Subscriptions will be called immediately from the thread that called this method.
   * Data cannot be null.
   *
   * @param data the data to set
   */
  public void set(T data) {
    if (data == null) throw new IllegalArgumentException("Data cannot be null");
    this.data = data;
    subscriptions.removeIf((Subscription subscription) -> !subscription.update(data));
  }

  /**
   * Subscribe to the data.
   *
   * @param consumer the consumer to call when the data is updated
   * @return a subscription object that can be used to destroy the subscription
   */
  public Subscription subscribe(Consumer<T> consumer) {
    return new Subscription(consumer);
  }

  /**
   * Subscribe to the data until the function returns true.
   *
   * @param function the function to call when the data is updated
   * @return a subscription object that can be used to destroy the subscription
   */
  public Subscription subscribeUntil(Function<T, Boolean> function) {
    return new Subscription(function);
  }

  /**
   * Subscribe to only the next update of the data.
   *
   * @param consumer the consumer to call when the data is updated
   * @return a subscription object that can be used to destroy the subscription
   */
  public Subscription subscribeOnce(Consumer<T> consumer) {
    return subscribeUntil(
        new Function<T, Boolean>() {
          private boolean called = false;

          @Override
          public Boolean apply(T t) {
            if (called) return true;
            called = true;
            consumer.accept(t);
            return false;
          }
        });
  }
}
