package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertIterableEquals;

import frc.UtilTest;
import java.util.ArrayList;
import java.util.List;

public class SharedReferenceTests {
  @UtilTest
  public void sharedReferenceStoresAutoBoxedPrimitive() {
    SharedReference<Integer> reference = new SharedReference<>(0);
    assertEquals(0, reference.get());
    reference.set(1);
    assertEquals(1, reference.get());
  }

  @UtilTest
  public void sharedReferenceStoresObject() {
    var list = new ArrayList<Integer>();
    SharedReference<List<Integer>> reference = new SharedReference<>(list);
    assertEquals(list, reference.get());

    list.add(1);
    assertEquals(list, reference.get());
    assertEquals(1, reference.get().size());
  }

  @UtilTest
  public void subscriptionsAreCalledWhenReferenceIsSet() {
    SharedReference<Integer> reference = new SharedReference<>(0);
    List<Integer> values = new ArrayList<>();
    var sub = reference.subscribe(values::add);
    reference.set(1);
    assertEquals(1, values.get(0));
    assertEquals(1, reference.get());

    reference.set(2);
    assertIterableEquals(List.of(1, 2), values);
    assertEquals(2, reference.get());

    reference.set(5);
    assertIterableEquals(List.of(1, 2, 5), values);
    assertEquals(5, reference.get());

    sub.destroy();

    reference.set(6);
    assertIterableEquals(List.of(1, 2, 5), values);
    assertEquals(6, reference.get());
  }
}
