package frc.util;

import java.util.function.Function;
import java.util.function.Supplier;

public class Any3<A, B, C> {

  enum Type {
    A,
    B,
    C
  }

  private final A a;
  private final B b;
  private final C c;

  private final Type type;

  private Any3(A a, B b, C c, Type type) {
    this.a = a;
    this.b = b;
    this.c = c;
    this.type = type;
  }

  public static <A, B, C> Any3<A, B, C> ofA(A a) {
    return new Any3<>(a, null, null, Type.A);
  }

  public static <A, B, C> Any3<A, B, C> ofB(B b) {
    return new Any3<>(null, b, null, Type.B);
  }

  public static <A, B, C> Any3<A, B, C> ofC(C c) {
    return new Any3<>(null, null, c, Type.C);
  }

  public <R> Supplier<R> map(Function<A, R> aFn, Function<B, R> bFn, Function<C, R> cFn) {
    return switch (type) {
      case A -> () -> aFn.apply(a);
      case B -> () -> bFn.apply(b);
      case C -> () -> cFn.apply(c);
    };
  }
}
