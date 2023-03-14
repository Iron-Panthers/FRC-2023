package frc.util;

import java.util.HashSet;
import java.util.Set;

public class CAN {
  private CAN() {}

  protected static final Set<Integer> ids = new HashSet<>();

  public static int at(int id) {
    ids.add(id);
    return id;
  }

  public static Set<Integer> getIds() {
    return ids;
  }
}
