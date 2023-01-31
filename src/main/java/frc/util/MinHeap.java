package frc.util;

import java.util.Objects;
import java.util.PriorityQueue;

public class MinHeap<T> {
  private PriorityQueue<Element<T>> heap = new PriorityQueue<>();

  public MinHeap() {
    // nothing to do
  }

  private static class Element<T> implements Comparable<Element<T>> {
    private T item;
    private double priority;

    public Element(T item, double priority) {
      this.item = item;
      this.priority = priority;
    }

    @Override
    public int compareTo(Element<T> other) {
      return Double.compare(priority, other.priority);
    }

    @Override
    @SuppressWarnings("unchecked")
    public boolean equals(Object other) {
      if (other == null) return false;
      if (other == this) return true;
      if (!(other instanceof Element<?>)) return false;

      Element<T> otherElement = (Element<T>) other;
      return priority == otherElement.priority && item.equals(otherElement.item);
    }

    @Override
    public int hashCode() {
      return Objects.hash(item, priority);
    }
  }

  public void add(T item, double priority) {
    heap.add(new Element<>(item, priority));
  }

  public T peakMin() {
    return heap.peek().item;
  }

  public T getMin() {
    return heap.poll().item;
  }

  public boolean isEmpty() {
    return heap.isEmpty();
  }
}
