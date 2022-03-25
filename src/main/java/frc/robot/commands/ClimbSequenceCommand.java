// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;

import static frc.robot.Constants.Climber.*;

import java.util.function.BooleanSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequenceCommand extends SequentialCommandGroup {
  /** Creates a new ClimbSequenceCommand. */
  public ClimbSequenceCommand(Climber climber, BooleanSupplier button) {
    addCommands(
      new InstantCommand(() -> climber.unlock()),
      new ExtendClimbToPosition(ExtendMovement.BOTTOM_TO_TOP, climber),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("Climb Sequence", "waiting");
        return button.getAsBoolean();}).andThen(
          () -> SmartDashboard.putString("Climb Sequence", "executing")),
      new ExtendClimbToPosition(ExtendMovement.TOP_TO_BOTTOM, climber),
      new ParallelRaceGroup(
        new RunCommand(
          () -> {
            climber.runAngle(TIMED_ANGLE_SPEED);
          }
        ).withTimeout(TIMED_ANGLE_DURATION).andThen(() -> climber.runAngle(0)),
        new ExtendClimbToPosition(ExtendMovement.HANG_BOTTOM, climber)
      ),
      new ExtendClimbToPosition(ExtendMovement.BOTTOM_TO_EVEN, climber),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("Climb Sequence", "waiting");
        return button.getAsBoolean();}).andThen(
          () -> SmartDashboard.putString("Climb Sequence", "executing")),
      new ExtendClimbToPosition(ExtendMovement.EVEN_TO_MID, climber).andThen(
        () -> SmartDashboard.putString("Climb Step", "A")),
      new AnglePIDCommand(climber, AngleMovement.MID_TO_MAX).andThen(
        () -> SmartDashboard.putString("Climb Step", "B")),
      new ExtendClimbToTop(climber).andThen(
        () -> SmartDashboard.putString("Climb Step", "C")),
      new AngleProfiledPIDCommand(AngleMovement.MAX_TO_HIGH, climber).andThen(
        () -> SmartDashboard.putString("Climb Step", "D")),
      new ParallelRaceGroup(
        new AnglePIDCommand(climber, AngleMovement.HOLD_HIGH),
        sequence(
          new WaitCommand(ANGLE_PID_PAUSE),
          new ExtendClimbToPosition(ExtendMovement.TOP_TO_HIGH, climber)
        )
      ).andThen(
        () -> SmartDashboard.putString("Climb Step", "E")),
      new ParallelRaceGroup(
        new RunCommand(
          () -> climber.runAngle(TIMED_ANGLE_SPEED)
        ).andThen(new InstantCommand(() -> climber.runAngle(0.0))),
        new ExtendClimbToPosition(ExtendMovement.HIGH_TO_BOTTOM, climber)
      ).andThen(
        () -> SmartDashboard.putString("Climb Step", "F")),
      parallel(
        new AnglePIDCommand(climber, AngleMovement.MAX_TO_HIGH_NO_LOAD),
        new ExtendClimbToPosition(ExtendMovement.BOTTOM_TO_MID, climber)
      ).andThen(
        () -> SmartDashboard.putString("Climb Step", "G")),
      parallel(
        new AnglePIDCommand(climber, AngleMovement.HIGH_TO_MIN),
        new ExtendClimbToPosition(ExtendMovement.MID_TO_BOTTOM, climber)
      ).andThen(
        () -> SmartDashboard.putString("Climb Step", "H")),
      new ParallelRaceGroup(
        new AngleClimbToPosition(climber, MID_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED),
        new ExtendClimbToPosition(ExtendMovement.HANG_BOTTOM, climber)
      ).andThen(
        () -> SmartDashboard.putString("Climb Step", "I")),
      new ExtendClimbToPosition(ExtendMovement.BOTTOM_TO_EVEN, climber).andThen(
        () -> SmartDashboard.putString("Climb Step", "J")),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("Climb Sequence", "waiting");
        return button.getAsBoolean();}).andThen(
          () -> SmartDashboard.putString("Climb Sequence", "executing")),
      new ExtendClimbToPosition(ExtendMovement.EVEN_TO_MID, climber).andThen(
        () -> SmartDashboard.putString("Climb Step", "A")),
      new AnglePIDCommand(climber, AngleMovement.MID_TO_MAX).andThen(
        () -> SmartDashboard.putString("Climb Step", "B")),
      new ExtendClimbToTop(climber).andThen(
        () -> SmartDashboard.putString("Climb Step", "C")),
      new AngleProfiledPIDCommand(AngleMovement.MAX_TO_HIGH, climber).andThen(
        () -> SmartDashboard.putString("Climb Step", "D")),
      new ParallelRaceGroup(
        new AnglePIDCommand(climber, AngleMovement.HOLD_HIGH),
        sequence(
          new WaitCommand(ANGLE_PID_PAUSE),
          new ExtendClimbToPosition(ExtendMovement.TOP_TO_HIGH, climber)
        )
      ).andThen(
        () -> SmartDashboard.putString("Climb Step", "E")),
      new ParallelRaceGroup(
        new RunCommand(
          () -> climber.runAngle(TIMED_ANGLE_SPEED)
        ).andThen(new InstantCommand(() -> climber.runAngle(0.0))),
        new ExtendClimbToPosition(ExtendMovement.HIGH_TO_BOTTOM, climber)
      ).andThen(
        () -> SmartDashboard.putString("Climb Step", "F")),
      parallel(
        new AnglePIDCommand(climber, AngleMovement.MAX_TO_HIGH_NO_LOAD),
        new ExtendClimbToPosition(ExtendMovement.BOTTOM_TO_MID, climber)
      ).andThen(
        () -> SmartDashboard.putString("Climb Step", "G")),
      parallel(
        new AnglePIDCommand(climber, AngleMovement.HIGH_TO_MIN),
        new ExtendClimbToPosition(ExtendMovement.MID_TO_BOTTOM, climber)
      ).andThen(
        () -> SmartDashboard.putString("Climb Step", "H")),
      new ParallelRaceGroup(
        new AngleClimbToPosition(climber, MID_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED),
        new ExtendClimbToPosition(ExtendMovement.HANG_BOTTOM, climber)
      ).andThen(
        () -> SmartDashboard.putString("Climb Step", "I")),
      new ExtendClimbToPosition(ExtendMovement.BOTTOM_TO_EVEN, climber).andThen(
        () -> SmartDashboard.putString("Climb Step", "J"))
    );
  }
}
