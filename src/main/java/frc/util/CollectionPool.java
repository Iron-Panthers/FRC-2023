package frc.util;

import java.util.ArrayDeque;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class CollectionPool<T> {
  private final ArrayDeque<T> pool = new ArrayDeque<>();
  private final Supplier<T> init;
  private final Consumer<T> reset;

  public CollectionPool(Supplier<T> init, Consumer<T> reset) {
    this.init = init;
    this.reset = reset;
  }

  public T get() {
    if (pool.isEmpty()) {
      return init.get();
    }
    return pool.pop();
  }

  public void put(T t) {
    reset.accept(t);
    pool.push(t);
  }
}
