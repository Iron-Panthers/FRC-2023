package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertIterableEquals;
import static org.junit.jupiter.api.Assertions.fail;

import frc.UtilTest;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

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

  @UtilTest
  public void subscriptionsExpireWhenTheyReturnTrue() {
    SharedReference<Integer> reference = new SharedReference<>(0);
    List<Integer> values = new ArrayList<>();
    var sub =
        reference.subscribeUntil(
            (Integer value) -> {
              values.add(value);
              return value < 5;
            });
    reference.set(1);
    assertEquals(1, values.get(0));
    assertEquals(1, reference.get());

    reference.set(2);
    assertIterableEquals(List.of(1, 2), values);
    assertEquals(2, reference.get());

    reference.set(5);
    assertIterableEquals(List.of(1, 2, 5), values);
    assertEquals(5, reference.get());

    reference.set(6);
    assertIterableEquals(List.of(1, 2, 5), values);
    assertEquals(6, reference.get());
  }

  @UtilTest
  public void subscribeOnceOnlySubscribesToNextUpdate() {
    SharedReference<Integer> reference = new SharedReference<>(0);
    List<Integer> values = new ArrayList<>();
    var sub = reference.subscribeOnce(values::add);
    reference.set(1);
    assertEquals(List.of(1), values);
    assertEquals(1, reference.get());

    reference.set(2);
    assertIterableEquals(List.of(1), values);
    assertEquals(2, reference.get());

    reference.set(5);
    assertIterableEquals(List.of(1), values);
    assertEquals(5, reference.get());
  }

  @UtilTest
  public void creatingASubscriptionDoesNotCallIt() {
    SharedReference<Integer> reference = new SharedReference<>(0);
    reference.subscribe(
        new Consumer<Integer>() {
          @Override
          public void accept(Integer value) {
            fail("Subscription should not be called when it is created");
          }
        });
  }

  @UtilTest
  public void creatingSubscriptionWithinSubscriptionDoesNotTriggerNewSubscription() {
    SharedReference<Integer> reference = new SharedReference<>(0);
    reference.subscribe(
        new Consumer<Integer>() {
          @Override
          public void accept(Integer value) {
            reference.subscribe(
                new Consumer<Integer>() {
                  @Override
                  public void accept(Integer value) {
                    fail(
                        "new subscription should not be called when it is created within subscription");
                  }
                });
          }
        });
    reference.set(1);
  }
}
