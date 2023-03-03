package frc.util;

import java.util.ArrayDeque;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.UnaryOperator;

/**
 * A class to facilitate a shared reference to data. It is best to use it with records, so the
 * reference will be set instead of mutated. This class is NOT THREAD SAFE AND WILL BLOW UP IN YOUR
 * FACE IF YOU PRETEND IT IS.
 */
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
  private ArrayDeque<Subscription> subscriptions = new ArrayDeque<>();

  public SharedReference(T data) {
    this.data = data;
  }

  private boolean doingUpdate = false;
  private ArrayDeque<Subscription> inflightAdditionalSubscriptions = new ArrayDeque<>();

  private void updateSubscriptions() {
    if (doingUpdate) {
      throw new IllegalStateException(
          "You are not allowed to call set from within a subscription.");
    }

    doingUpdate = true;

    subscriptions.removeIf(sub -> !sub.update(data));

    subscriptions.addAll(inflightAdditionalSubscriptions);
    inflightAdditionalSubscriptions.clear();

    doingUpdate = false;
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
   * Data cannot be null. Setting the data inside a subscription will throw a {@link
   * IllegalStateException}. Setting the data to the same value <i><b>will call
   * subscriptions.</b></i>
   *
   * @param data the data to set
   */
  public void set(T data) {
    if (data == null) throw new IllegalArgumentException("Data cannot be null");
    this.data = data;
    updateSubscriptions();
  }

  /**
   * Apply a unary operator to the data. Subscriptions will be called immediately from the thread
   * that called this method. Equivalent to calling {@link #set} with the result of {@link
   * UnaryOperator#apply} on {@link #get}.
   *
   * @param function the function to apply
   */
  public void apply(UnaryOperator<T> function) {
    set(function.apply(data));
  }

  /**
   * Subscribe to the data until the function returns true. All other subscriptions call this method
   * internally.
   *
   * @param function the function to call when the data is updated
   * @return a subscription object that can be used to destroy the subscription
   */
  public Subscription subscribeUntil(Function<T, Boolean> function) {
    var sub = new Subscription(function);
    if (doingUpdate) {
      inflightAdditionalSubscriptions.add(sub);
    } else {
      subscriptions.add(sub);
    }
    return sub;
  }

  /**
   * Subscribe to the data.
   *
   * @param consumer the consumer to call when the data is updated
   * @return a subscription object that can be used to destroy the subscription
   */
  public Subscription subscribe(Consumer<T> consumer) {
    return subscribeUntil(
        u -> {
          consumer.accept(u);
          return true;
        });
  }

  /**
   * Subscribe to the data.
   *
   * @param runnable the runnable to call when the data is updated
   * @return a subscription object that can be used to destroy the subscription
   */
  public Subscription subscribe(Runnable runnable) {
    return subscribe(v -> runnable.run());
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

  /**
   * Subscribe to only the next update of the data.
   *
   * @param runnable the runnable to call when the data is updated
   * @return a subscription object that can be used to destroy the subscription
   */
  public Subscription subscribeOnce(Runnable runnable) {
    return subscribeOnce(v -> runnable.run());
  }
}
