// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class HashMapCommand<T> extends CommandBase {
  private final Map<T, Command> commandMap;
  private final Supplier<T> keySupplier;
  private Command selectedCommand;
  private boolean runsWhenDisabled;

  /**
   * Creates a new ConditionalCommand.
   *
   * @param onTrue the command to run if the condition is true
   * @param onFalse the command to run if the condition is false
   * @param condition the condition to determine which command to run
   */
  public HashMapCommand(Map<T, Command> commandMap, Supplier<T> keySupplier) {
    this.commandMap = commandMap;
    this.keySupplier = keySupplier;

    CommandScheduler.getInstance()
        .registerComposedCommands(commandMap.values().toArray(new Command[0]));

    m_requirements.addAll(
        commandMap.values().stream()
            .flatMap(command -> command.getRequirements().stream())
            .collect(Collectors.toList()));

    runsWhenDisabled = commandMap.values().stream().allMatch(Command::runsWhenDisabled);
  }

  @Override
  public void initialize() {
    var key = keySupplier.get();
    selectedCommand = commandMap.get(key);
    if (selectedCommand == null) {
      throw new IllegalArgumentException("No command for key " + key);
    }
    selectedCommand.initialize();
  }

  @Override
  public void execute() {
    selectedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    selectedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return selectedCommand.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return runsWhenDisabled;
  }
}
