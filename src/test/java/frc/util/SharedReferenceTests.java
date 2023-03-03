package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertIterableEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
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

  @UtilTest
  public void creatingSubscriptionWithinSubscriptionTriggersOnNextUpdate() {
    SharedReference<Integer> reference = new SharedReference<>(0);
    List<Integer> valuesA = new ArrayList<>();
    List<Integer> valuesB = new ArrayList<>();
    reference.subscribe(valuesA::add);
    reference.subscribeOnce(v -> reference.subscribe(valuesB::add));

    assertIterableEquals(List.of(), valuesA);
    assertIterableEquals(List.of(), valuesB);

    reference.set(1);
    assertEquals(List.of(1), valuesA);
    assertEquals(List.of(), valuesB);

    reference.set(2);

    assertEquals(List.of(1, 2), valuesA);
    assertEquals(List.of(2), valuesB);
  }

  @UtilTest
  public void callingSetInsideSubscriptionThrowsError() {
    SharedReference<Integer> reference = new SharedReference<>(0);
    reference.subscribe(
        v -> {
          assertThrows(IllegalStateException.class, () -> reference.set(1));
        });
    reference.set(1);
  }

  private static class ExpiringData {
    private boolean isExpired = false;
    public final int value;

    public ExpiringData(int value) {
      this.value = value;
    }

    public void expire() {
      isExpired = true;
    }
  }

  @UtilTest
  public void subscribeOnceWithinSubscriptionCleansUpObject() {
    SharedReference<Integer> reference = new SharedReference<>(0);
    List<Integer> values = new ArrayList<>();
    List<ExpiringData> dataList = new ArrayList<>();

    reference.subscribe(
        v -> {
          ExpiringData data = new ExpiringData(v);
          dataList.add(data);
          reference.subscribeOnce(v2 -> data.expire());
          values.add(v);
        });

    assertIterableEquals(List.of(), dataList, "No data should be created as a reaction yet");
    assertIterableEquals(
        List.of(), values, "No values should have been given to subscriptions yet");

    reference.set(1);
    assertEquals(1, dataList.size(), "One data object should have been created");
    assertIterableEquals(List.of(1), values, "One value should have been given to subscription");
    assertFalse(dataList.get(0).isExpired, "Data object should not have expired yet");

    reference.set(2);
    assertEquals(2, dataList.size(), "Two data objects should have been created");
    assertIterableEquals(
        List.of(1, 2), values, "Two values should have been given to subscription");
    assertTrue(dataList.get(0).isExpired, "First data object should have expired");
    assertFalse(dataList.get(1).isExpired, "Second data object should not have expired yet");
  }

  @UtilTest
  public void subscribeOnceWithinSubscriptionUsingRunnableCleansUpObject() {
    SharedReference<Integer> reference = new SharedReference<>(0);
    List<Integer> values = new ArrayList<>();
    List<ExpiringData> dataList = new ArrayList<>();

    reference.subscribe(
        v -> {
          ExpiringData data = new ExpiringData(v);
          dataList.add(data);
          reference.subscribeOnce(data::expire);
          values.add(v);
        });

    assertIterableEquals(List.of(), dataList, "No data should be created as a reaction yet");
    assertIterableEquals(
        List.of(), values, "No values should have been given to subscriptions yet");

    reference.set(1);
    assertEquals(1, dataList.size(), "One data object should have been created");
    assertIterableEquals(List.of(1), values, "One value should have been given to subscription");
    assertFalse(dataList.get(0).isExpired, "Data object should not have expired yet");

    reference.set(2);
    assertEquals(2, dataList.size(), "Two data objects should have been created");
    assertIterableEquals(
        List.of(1, 2), values, "Two values should have been given to subscription");
    assertTrue(dataList.get(0).isExpired, "First data object should have expired");
    assertFalse(dataList.get(1).isExpired, "Second data object should not have expired yet");
  }
}
