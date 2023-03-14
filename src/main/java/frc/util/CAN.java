package frc.util;

import java.util.ArrayList;
import java.util.List;

public class CAN {
  private CAN() {}

  protected static final List<Integer> ids = new ArrayList<>();

  public static int at(int id) {
    ids.add(id);
    return id;
  }

  public static List<Integer> getIds() {
    return ids;
  }
}
