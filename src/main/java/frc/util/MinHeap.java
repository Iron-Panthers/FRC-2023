package frc.util;

import java.util.PriorityQueue;

public class MinHeap<T> {
  private PriorityQueue<Element<T>> heap = new PriorityQueue<>();

  public MinHeap() {
    // nothing to do
  }

  /** Note: this class has a natural ordering that is inconsistent with equals. */
  private static class Element<T> implements Comparable<Element<T>> {
    private T item;
    private int priority;

    public Element(T item, int priority) {
      this.item = item;
      this.priority = priority;
    }

    @Override
    public int compareTo(Element<T> other) {
      return priority - other.priority;
    }

    @Override
    @SuppressWarnings("unchecked")
    public boolean equals(Object other) {
      if (other == null) return false;
      if (other == this) return true;
      if (!(other instanceof Element<?>)) return false;

      Element<T> otherElement = (Element<T>) other;
      return item.equals(otherElement.item);
    }

    @Override
    public int hashCode() {
      // dirty, evil, sinful hack
      return item.hashCode();
    }
  }

  public void add(T item, int priority) {
    heap.add(new Element<>(item, priority));
  }

  public T peakMin() {
    return heap.peek().item;
  }

  public T getMin() {
    return heap.poll().item;
  }

  /**
   * Determine if the heap has an item. Priority is not considered.
   *
   * @param item The item to check for.
   * @return True if the item is in the heap, false otherwise.
   */
  public boolean contains(T item) {
    return heap.contains(new Element<>(item, 0));
  }

  public boolean isEmpty() {
    return heap.isEmpty();
  }

  public void clear() {
    heap.clear();
  }
}
