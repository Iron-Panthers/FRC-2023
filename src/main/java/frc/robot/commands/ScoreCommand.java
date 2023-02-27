// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.OuttakeSubsystem;
import java.util.Optional;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCommand extends SequentialCommandGroup {
  record ScoreStep(Optional<ArmState> armState, Optional<OuttakeSubsystem.Modes> outtakeState) {
    public ScoreStep(ArmState armState, OuttakeSubsystem.Modes outtakeState) {
      this(Optional.of(armState), Optional.of(outtakeState));
    }

    public ScoreStep(ArmState armState) {
      this(Optional.of(armState), Optional.empty());
    }

    public ScoreStep(OuttakeSubsystem.Modes outtakeState) {
      this(Optional.empty(), Optional.of(outtakeState));
    }
  }

  /** Creates a new ScoreCommand. */
  public ScoreCommand(
      OuttakeSubsystem outtakeSubsystem, ArmSubsystem armSubsystem, ScoreStep[] scoreSteps) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    for (ScoreStep scoreStep : scoreSteps) {
      var armState = scoreStep.armState();
      var outtakeState = scoreStep.outtakeState();
      if (armState.isPresent() && outtakeState.isPresent()) {
        addCommands(
            new ArmPositionCommand(armSubsystem, armState.get())
                .deadlineWith(new SetOuttakeModeCommand(outtakeSubsystem, outtakeState.get())));
      } else if (armState.isPresent()) {
        addCommands(new ArmPositionCommand(armSubsystem, armState.get()));
      } else if (outtakeState.isPresent()) {
        addCommands(new SetOuttakeModeCommand(outtakeSubsystem, outtakeState.get()));
      } else {
        throw new IllegalArgumentException("ScoreStep must have at least one state");
      }
    }
  }
}
