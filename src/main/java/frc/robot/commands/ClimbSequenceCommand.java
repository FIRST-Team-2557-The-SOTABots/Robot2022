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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climber;

import static frc.robot.Constants.Climber.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequenceCommand extends SequentialCommandGroup {
  /** Creates a new ClimbSequenceCommand. */
  public ClimbSequenceCommand(Climber climber, JoystickButton button) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> climber.unlock()),
      new ExtendClimbToPosition(ExtendMovement.BOTTOM_TO_TOP, climber),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("waiting", "step 1");
        return button.get();}),
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
        SmartDashboard.putString("waiting", "step 2");
        return button.get();}),
      new ExtendClimbToPosition(ExtendMovement.EVEN_TO_MID, climber),
      new AnglePIDCommand(climber, AngleMovement.MID_TO_MAX),
      new ExtendClimbToPosition(ExtendMovement.MID_TO_TOP, climber).withTimeout(ANGLED_EXTEND_TIMEOUT),
      new AngleProfiledPIDCommand(AngleMovement.MAX_TO_HIGH, climber),
      new ParallelRaceGroup(
        new AnglePIDCommand(climber, AngleMovement.HOLD_HIGH),
        sequence(
          new WaitCommand(ANGLE_PID_PAUSE),
          new ExtendClimbToPosition(ExtendMovement.TOP_TO_HIGH, climber)
        )
      ),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("waiting", "step 3");
        return button.get();}),
      new ParallelRaceGroup(
        new RunCommand(
          () -> climber.runAngle(TIMED_ANGLE_SPEED)
        ).andThen(new InstantCommand(() -> climber.runAngle(0.0))),
        new ExtendClimbToPosition(ExtendMovement.HIGH_TO_BOTTOM, climber)
      ),
      parallel(
        new AnglePIDCommand(climber, AngleMovement.MAX_TO_HIGH_NO_LOAD),
        new ExtendClimbToPosition(ExtendMovement.BOTTOM_TO_MID, climber)
      ),
      parallel(
        new AnglePIDCommand(climber, AngleMovement.HIGH_TO_MIN),
        new ExtendClimbToPosition(ExtendMovement.MID_TO_BOTTOM, climber)
      ),
      new ParallelRaceGroup(
        new RunCommand(
          () -> {
            climber.runAngle(TIMED_ANGLE_SPEED);
          }
        ).withTimeout(TIMED_ANGLE_DURATION_2).andThen(() -> climber.runAngle(0)),
        new ExtendClimbToPosition(ExtendMovement.HANG_BOTTOM, climber)
      ),
      new ExtendClimbToPosition(ExtendMovement.BOTTOM_TO_EVEN, climber),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("waiting", "step 2");
        return button.get();}),
      new ExtendClimbToPosition(ExtendMovement.EVEN_TO_MID, climber),
      new AnglePIDCommand(climber, AngleMovement.MID_TO_MAX),
      new ExtendClimbToPosition(ExtendMovement.MID_TO_TOP, climber).withTimeout(ANGLED_EXTEND_TIMEOUT),
      new AngleProfiledPIDCommand(AngleMovement.MAX_TO_HIGH, climber),
      new ParallelRaceGroup(
        new AnglePIDCommand(climber, AngleMovement.HOLD_HIGH),
        sequence(
          new WaitCommand(ANGLE_PID_PAUSE),
          new ExtendClimbToPosition(ExtendMovement.TOP_TO_HIGH, climber)
        )
      ),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("waiting", "step 3");
        return button.get();}),
      new ParallelRaceGroup(
        new RunCommand(
          () -> climber.runAngle(TIMED_ANGLE_SPEED)
        ).andThen(new InstantCommand(() -> climber.runAngle(0.0))),
        new ExtendClimbToPosition(ExtendMovement.HIGH_TO_BOTTOM, climber)
      ),
      parallel(
        new AnglePIDCommand(climber, AngleMovement.MAX_TO_HIGH_NO_LOAD),
        new ExtendClimbToPosition(ExtendMovement.BOTTOM_TO_MID, climber)
      ),
      parallel(
        new AnglePIDCommand(climber, AngleMovement.HIGH_TO_MIN),
        new ExtendClimbToPosition(ExtendMovement.MID_TO_BOTTOM, climber)
      ),
      new ParallelRaceGroup(
        new RunCommand(
          () -> {
            climber.runAngle(TIMED_ANGLE_SPEED);
          }
        ).withTimeout(TIMED_ANGLE_DURATION_2).andThen(() -> climber.runAngle(0)),
        new ExtendClimbToPosition(ExtendMovement.HANG_BOTTOM, climber)
      ),
      new ExtendClimbToPosition(ExtendMovement.BOTTOM_TO_EVEN, climber)
    );
  }
}
