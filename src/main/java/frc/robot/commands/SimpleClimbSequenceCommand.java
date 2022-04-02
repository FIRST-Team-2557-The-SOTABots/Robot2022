// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static frc.robot.Constants.Climber.*;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleClimbSequenceCommand extends SequentialCommandGroup {
  /** Creates a new SimpleClimbSequenceCommand. */
  public SimpleClimbSequenceCommand(Climber climber, BooleanSupplier button) {
    addCommands(
      new ExtendClimbCommand(climber, SimpleExtendMovement.BOTTOM_TO_TOP),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("Climb Sequence", "waiting");
        return button.getAsBoolean();}).andThen(
          () -> SmartDashboard.putString("Climb Sequence", "executing")),
      new InstantCommand(() -> climber.unlock()),
      new ExtendClimbCommand(climber, SimpleExtendMovement.TOP_TO_BOTTOM),
      new AngleClimbToPosition(climber, MID_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED).withTimeout(ANGLE_HOOKS_TO_BAR_TIMEOUT),
      new ExtendClimbCommand(climber, SimpleExtendMovement.BOTTOM_TO_EVEN),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("Climb Sequence", "waiting");
        return button.getAsBoolean();}).andThen(
          () -> SmartDashboard.putString("Climb Sequence", "executing")),
      new ExtendClimbCommand(climber, SimpleExtendMovement.EVEN_TO_MID),
      new AnglePIDCommand(climber, AngleMovement.MID_TO_MAX),
      new ExtendClimbCommand(climber, SimpleExtendMovement.MID_TO_TOP),
      new AngleProfiledPIDCommand(AngleMovement.MAX_TO_HIGH, climber),
      race(
        new AnglePIDCommand(climber, AngleMovement.HOLD_HIGH),
        new ExtendClimbCommand(climber, SimpleExtendMovement.TOP_TO_HIGH)
      ),
      parallel(
        new AngleClimbToPosition(climber, MAX_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED),
        new ExtendClimbCommand(climber, SimpleExtendMovement.HIGH_TO_BOTTOM)
      ),
      new AngleClimbToPosition(climber, HAX_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED_FAST),
      new ExtendClimbCommand(climber, SimpleExtendMovement.BOTTOM_TO_MID),
      new AngleClimbToPosition(climber, MIN_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED_FAST),
      new ExtendClimbCommand(climber, SimpleExtendMovement.MID_TO_BOTTOM),
      new AngleClimbToPosition(climber, MID_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED).withTimeout(ANGLE_HOOKS_TO_BAR_TIMEOUT),
      new ExtendClimbCommand(climber, SimpleExtendMovement.BOTTOM_TO_EVEN),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("Climb Sequence", "waiting");
        return button.getAsBoolean();}).andThen(
          () -> SmartDashboard.putString("Climb Sequence", "executing")),
      new ExtendClimbCommand(climber, SimpleExtendMovement.EVEN_TO_MID),
      new AnglePIDCommand(climber, AngleMovement.MID_TO_MAX),
      new ExtendClimbCommand(climber, SimpleExtendMovement.MID_TO_TOP),
      new AngleProfiledPIDCommand(AngleMovement.MAX_TO_HIGH, climber),
      race(
        new AnglePIDCommand(climber, AngleMovement.HOLD_HIGH),
        new ExtendClimbCommand(climber, SimpleExtendMovement.TOP_TO_HIGH)
      ),
      parallel(
        new AngleClimbToPosition(climber, MAX_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED),
        new ExtendClimbCommand(climber, SimpleExtendMovement.HIGH_TO_BOTTOM)
      ),
      new AngleClimbToPosition(climber, HAX_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED_FAST),
      new ExtendClimbCommand(climber, SimpleExtendMovement.BOTTOM_TO_MID),
      new AngleClimbToPosition(climber, MIN_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED_FAST),
      new ExtendClimbCommand(climber, SimpleExtendMovement.MID_TO_BOTTOM),
      new AngleClimbToPosition(climber, MID_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED).withTimeout(ANGLE_HOOKS_TO_BAR_TIMEOUT),
      new ExtendClimbCommand(climber, SimpleExtendMovement.BOTTOM_TO_EVEN)
    );
  }
}
