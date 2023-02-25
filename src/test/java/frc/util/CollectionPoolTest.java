package frc.util;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.UtilTest;
import java.util.function.Supplier;

public class CollectionPoolTest {
  @UtilTest
  public void collectionPoolConstructs() {
    assertDoesNotThrow(() -> new CollectionPool<>(() -> null, t -> {}));
  }

  private static class FakeCollection {
    public int id = 0;
    public int resetCount = 0;

    public FakeCollection(int id) {
      this.id = id;
    }

    public void reset() {
      resetCount++;
    }
  }

  private static Supplier<FakeCollection> createIncrementingSupplier() {
    return new Supplier<FakeCollection>() {
      private int id = 0;

      @Override
      public FakeCollection get() {
        return new FakeCollection(id++);
      }
    };
  }

  @UtilTest
  public void collectionPoolGetsUsingConstructor() {
    var supplier = createIncrementingSupplier();
    CollectionPool<FakeCollection> pool = new CollectionPool<>(supplier, FakeCollection::reset);

    var collection0 = pool.get();
    assertEquals(0, collection0.id);
    assertEquals(0, collection0.resetCount);
    var collection1 = pool.get();
    assertEquals(1, collection1.id);
    assertEquals(0, collection1.resetCount);
    var collection2 = pool.get();
    assertEquals(2, collection2.id);
    assertEquals(0, collection2.resetCount);
  }

  @UtilTest
  public void collectionPoolPuts() {
    var supplier = createIncrementingSupplier();
    CollectionPool<FakeCollection> pool = new CollectionPool<>(supplier, FakeCollection::reset);

    var collection0 = pool.get();
    var collection1 = pool.get();
    var collection2 = pool.get();

    pool.put(collection1);

    var collection3 = pool.get();
    assertEquals(1, collection3.id);
    assertEquals(1, collection3.resetCount);

    pool.put(collection3);

    var collection4 = pool.get();
    assertEquals(1, collection4.id);
    assertEquals(2, collection4.resetCount);

    assertEquals(collection1, collection4);
  }

  @UtilTest
  public void collectionPoolOnlyConstructsIfEmpty() {
    var supplier = createIncrementingSupplier();
    CollectionPool<FakeCollection> pool = new CollectionPool<>(supplier, FakeCollection::reset);

    var collection0 = pool.get();
    var collection1 = pool.get();
    var collection2 = pool.get();

    pool.put(collection0);
    pool.put(collection1);
    pool.put(collection2);

    var collection3 = pool.get();
    var collection4 = pool.get();
    var collection5 = pool.get();

    assertEquals(collection0, collection5);
    assertEquals(collection1, collection4);
    assertEquals(collection2, collection3);
  }
}
