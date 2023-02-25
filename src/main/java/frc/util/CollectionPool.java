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
    var collection = pool.pop();
    reset.accept(collection);
    return collection;
  }

  public void put(T t) {
    // we will wait to reset the collection until it is popped from the pool
    pool.push(t);
  }
}
