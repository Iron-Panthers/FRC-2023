// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.OuttakeSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCommand extends SequentialCommandGroup {
  public static record ScoreStep(
      Optional<ArmState> armState,
      Optional<OuttakeSubsystem.Modes> outtakeState,
      boolean isPausePoint) {
    public ScoreStep(ArmState armState, OuttakeSubsystem.Modes outtakeState) {
      this(Optional.of(armState), Optional.of(outtakeState), false);
    }

    public ScoreStep(ArmState armState) {
      this(Optional.of(armState), Optional.empty(), false);
    }

    public ScoreStep(OuttakeSubsystem.Modes outtakeState) {
      this(Optional.empty(), Optional.of(outtakeState), false);
    }

    public ScoreStep canWaitHere() {
      return new ScoreStep(armState, outtakeState, true);
    }
  }

  private static class AwaitTriggerPressed extends CommandBase {
    private final Trigger trigger;
    private boolean hasBeenFalse;

    public AwaitTriggerPressed(Trigger trigger) {
      this.trigger = trigger;
    }

    @Override
    public void initialize() {
      hasBeenFalse = false;
    }

    @Override
    public void execute() {
      if (!trigger.getAsBoolean()) {
        hasBeenFalse = true;
      }
    }

    @Override
    public boolean isFinished() {
      return hasBeenFalse && trigger.getAsBoolean();
    }
  }

  private Command createStep(ScoreStep scoreStep) {
    var armState = scoreStep.armState();
    var outtakeState = scoreStep.outtakeState();
    if (armState.isPresent() && outtakeState.isPresent()) {
      return new ArmPositionCommand(armSubsystem, armState.get())
          .deadlineWith(new SetOuttakeModeCommand(outtakeSubsystem, outtakeState.get()));
    } else if (armState.isPresent()) {
      return new ArmPositionCommand(armSubsystem, armState.get());
    } else if (outtakeState.isPresent()) {
      return new SetOuttakeModeCommand(outtakeSubsystem, outtakeState.get());
    } else {
      throw new IllegalArgumentException("ScoreStep must have at least one state");
    }
  }

  private final ArmSubsystem armSubsystem;
  private final OuttakeSubsystem outtakeSubsystem;

  public ScoreCommand(
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      List<ScoreStep> scoreSteps,
      Trigger trigger) {
    this(outtakeSubsystem, armSubsystem, scoreSteps, Optional.of(trigger), Optional.empty());
  }

  public ScoreCommand(
      OuttakeSubsystem outtakeSubsystem, ArmSubsystem armSubsystem, List<ScoreStep> scoreSteps) {
    this(outtakeSubsystem, armSubsystem, scoreSteps, Optional.empty(), Optional.empty());
  }

  public ScoreCommand(
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      List<ScoreStep> scoreSteps,
      double stepDeadline) {
    this(outtakeSubsystem, armSubsystem, scoreSteps, Optional.empty(), Optional.of(stepDeadline));
  }

  /** Creates a new ScoreCommand. */
  private ScoreCommand(
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      List<ScoreStep> scoreSteps,
      Optional<Trigger> trigger,
      Optional<Double> stepDeadline) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.outtakeSubsystem = outtakeSubsystem;
    this.armSubsystem = armSubsystem;

    for (ScoreStep scoreStep : scoreSteps) {
      if (trigger.isEmpty())
        addCommands(
            stepDeadline.isPresent()
                ? createStep(scoreStep).withTimeout(stepDeadline.get())
                : createStep(scoreStep));
      else
        addCommands(
            scoreStep.isPausePoint()
                ? (new AwaitTriggerPressed(trigger.get())).deadlineWith(createStep(scoreStep))
                : (new AwaitTriggerPressed(trigger.get())).raceWith(createStep(scoreStep)));
    }
  }

  public static List<ScoreCommand> splitAlongPausePoints(
      OuttakeSubsystem outtakeSubsystem,
      ArmSubsystem armSubsystem,
      List<ScoreStep> scoreSteps,
      double stepDeadline) {
    var scoreCommands = new ArrayList<ScoreCommand>();

    int start = 0;
    int end = 0;
    while (end < scoreSteps.size()) {
      if (scoreSteps.get(end).isPausePoint()) {
        // System.out.printf("start: %d end: %d%n", start, end);
        // System.out.println(scoreSteps.subList(start, end + 1));
        scoreCommands.add(
            new ScoreCommand(
                outtakeSubsystem, armSubsystem, scoreSteps.subList(start, end + 1), stepDeadline));
        start = end + 1;
      }
      end++;
    }

    // System.out.printf("start: %d end: %d%n", start, end);
    // System.out.println(scoreSteps.subList(start, end));
    scoreCommands.add(
        new ScoreCommand(
            outtakeSubsystem, armSubsystem, scoreSteps.subList(start, end), stepDeadline));

    return scoreCommands;
  }
}
