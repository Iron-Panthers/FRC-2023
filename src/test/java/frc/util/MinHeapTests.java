package frc.util;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.UtilTest;

public class MinHeapTests {

  enum Items {
    A,
    B,
    C,
    D,
    E,
    F
  }

  @UtilTest
  public void constructsMinHeap() {
    assertDoesNotThrow(() -> new MinHeap<Items>());
  }

  @UtilTest
  public void addsToMinHeap() {
    assertDoesNotThrow(
        () -> {
          MinHeap<Items> heap = new MinHeap<>();
          heap.add(Items.A, 1);
          heap.add(Items.B, 2);
          heap.add(Items.C, 3);
          heap.add(Items.D, 4);
          heap.add(Items.E, 5);
          heap.add(Items.F, 6);
        });
  }

  @UtilTest
  public void getsMinFromMinHeap() {
    MinHeap<Items> heap = new MinHeap<>();
    heap.add(Items.A, 1);
    heap.add(Items.B, 2);

    assertEquals(Items.A, heap.getMin());
    assertEquals(Items.B, heap.getMin());

    heap.add(Items.C, 3);

    assertEquals(Items.C, heap.getMin());

    assertTrue(heap.isEmpty());
  }

  @UtilTest
  public void peaksMinFromMinHeap() {
    MinHeap<Items> heap = new MinHeap<>();
    heap.add(Items.A, 1);
    heap.add(Items.B, 2);

    assertEquals(Items.A, heap.peakMin());
    assertEquals(Items.A, heap.peakMin());

    heap.add(Items.C, 3);

    assertEquals(Items.A, heap.peakMin());

    heap.add(Items.D, -100);

    assertEquals(Items.D, heap.peakMin());

    assertFalse(heap.isEmpty());

    assertEquals(Items.D, heap.getMin());
    assertEquals(Items.A, heap.getMin());
    assertEquals(Items.B, heap.getMin());
    assertEquals(Items.C, heap.getMin());

    assertTrue(heap.isEmpty());
  }

  @UtilTest
  public void addingSameElementAgainOverwritesOld() {
    MinHeap<Items> heap = new MinHeap<>();

    heap.add(Items.A, 1);
    heap.add(Items.B, 2);

    heap.add(Items.C, 3);

    assertEquals(Items.A, heap.peakMin());

    heap.add(Items.C, -2);

    assertEquals(Items.C, heap.getMin());

    assertEquals(Items.A, heap.getMin());
    assertEquals(Items.B, heap.getMin());
    assertEquals(Items.C, heap.getMin());

    assertTrue(heap.isEmpty());

    assertThrows(NullPointerException.class, () -> heap.getMin());
  }
}
