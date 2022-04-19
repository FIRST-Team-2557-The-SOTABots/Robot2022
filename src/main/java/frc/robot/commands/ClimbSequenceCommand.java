// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static frc.robot.Constants.Climber.*;
import frc.robot.subsystems.Climber;

public class ClimbSequenceCommand extends SequentialCommandGroup {
  /** Creates a new SimpleClimbSequenceCommand. */
  public ClimbSequenceCommand(Climber climber, BooleanSupplier button) {
    addCommands(
      // Step 1: extend the climb hooks to the top, then wait for a button press
      new ExtendClimbCommand(climber, SimpleExtendMovement.BOTTOM_TO_TOP),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("Climb Sequence", "waiting");
        return button.getAsBoolean();}).andThen(
          () -> SmartDashboard.putString("Climb Sequence", "executing")),

      // Step 2: unlock the angle motor, completely retract the hooks to bring the robot up
      // angle the climb to even with the bar, giving up if the command has not completed in a certain amount of time
      // then lower the angle hooks onto the bar by extending the hooks slightly
      // finally wait for a button press
      new InstantCommand(() -> climber.unlock()),
      new ExtendClimbCommand(climber, SimpleExtendMovement.TOP_TO_BOTTOM),
      new AngleClimbToPosition(climber, MID_ANGLE_ENCODER, RUN_TO_BAR_SPEED).withTimeout(ANGLE_HOOKS_TO_BAR_TIMEOUT),
      deadline(
        sequence(
          new WaitCommand(LOWER_TO_BAR_DELAY),
          new ExtendClimbCommand(climber, SimpleExtendMovement.BOTTOM_TO_EVEN)
        ),
        new AngleClimbToPosition(climber, MID_ANGLE_ENCODER, RUN_TO_BAR_SPEED, RUN_TO_BAR_TOLERANCE, RUN_TO_ANGLE_BAR_END)
      ),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("Climb Sequence", "waiting");
        return button.getAsBoolean();}).andThen(
          () -> SmartDashboard.putString("Climb Sequence", "executing")),

      // Step 3.1: raise the extend hooks off the bar so the robot can angle completely backwards,
      // then extend the hooks completely
      // angle the robot forward slightly, then hold there as the extend hooks retract slightly onto the bar
      new ExtendClimbCommand(climber, SimpleExtendMovement.EVEN_TO_MID),
      parallel(
        new AnglePIDCommand(climber, AngleMovement.MID_TO_MAX),
        new ExtendClimbCommand(climber, SimpleExtendMovement.MID_TO_TOP)
      ),
      race(
        new AngleProfiledPIDCommand(AngleMovement.MAX_TO_HIGH, climber),
        new WaitUntilCommand(climber::getStallProtectionOn)
      ),
      race(
        new AnglePIDCommand(climber, AngleMovement.HOLD_HIGH),
        new ExtendClimbCommand(climber, SimpleExtendMovement.TOP_TO_HIGH)
      ),

      // Step 3.2: as the hooks retract and raise the robot completely up, angle the hooks forward to prevent swinging
      // the angle hooks can then be taken slightly off the bar,
      // extend the hooks to lower the robot and allow the angle hooks to pass under,
      // retract the hooks to raise the robot and angle the hooks onto the bar, then lower the angle hooks onto the bar
      // finally wait for a button press
      parallel(
        new AngleClimbToPosition(climber, MAX_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED),
        new ExtendClimbCommand(climber, SimpleExtendMovement.HIGH_TO_RELEASE)
      ),
      deadline(
        new AngleClimbToPosition(climber, MIN_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED_FAST, RUN_TO_ANGLE_TOLERANCE_FAST), 
        new ExtendClimbCommand(climber, SimpleExtendMovement.HOLD_RELEASE)
      ),
      new ExtendClimbCommand(climber, SimpleExtendMovement.RELEASE_TO_BOTTOM),
      new AngleClimbToPosition(climber, MID_ANGLE_ENCODER, RUN_TO_BAR_SPEED).withTimeout(ANGLE_HOOKS_TO_BAR_TIMEOUT),
      deadline(
        sequence(
          new WaitCommand(LOWER_TO_BAR_DELAY),
          new ExtendClimbCommand(climber, SimpleExtendMovement.BOTTOM_TO_EVEN)
        ),
        new AngleClimbToPosition(climber, MID_ANGLE_ENCODER, RUN_TO_BAR_SPEED, RUN_TO_BAR_TOLERANCE, RUN_TO_ANGLE_BAR_END)
      ),
      new WaitUntilCommand(() -> {
        SmartDashboard.putString("Climb Sequence", "waiting");
        return button.getAsBoolean();}).andThen(
          () -> SmartDashboard.putString("Climb Sequence", "executing")),

      // Step 3.1: raise the extend hooks off the bar so the robot can angle completely backwards,
      // then extend the hooks completely
      // angle the robot forward slightly, then hold there as the extend hooks retract slightly onto the bar
      new ExtendClimbCommand(climber, SimpleExtendMovement.EVEN_TO_MID),
      parallel(
        new AnglePIDCommand(climber, AngleMovement.MID_TO_MAX),
        new ExtendClimbCommand(climber, SimpleExtendMovement.MID_TO_TOP)
      ),
      race(
        new AngleProfiledPIDCommand(AngleMovement.MAX_TO_HIGH, climber),
        new WaitUntilCommand(climber::getStallProtectionOn)
      ),
      race(
        new AnglePIDCommand(climber, AngleMovement.HOLD_HIGH),
        new ExtendClimbCommand(climber, SimpleExtendMovement.TOP_TO_HIGH)
      ),

      // Step 3.2: as the hooks retract and raise the robot completely up, angle the hooks forward to prevent swinging
      // the angle hooks can then be taken slightly off the bar,
      // extend the hooks to lower the robot and allow the angle hooks to pass under,
      // retract the hooks to raise the robot and angle the hooks onto the bar, then lower the angle hooks onto the bar
      parallel(
        new AngleClimbToPosition(climber, MAX_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED),
        new ExtendClimbCommand(climber, SimpleExtendMovement.HIGH_TO_RELEASE)
      ),
      deadline(
        new AngleClimbToPosition(climber, MIN_ANGLE_ENCODER, RUN_TO_ANGLE_SPEED_FAST, RUN_TO_ANGLE_TOLERANCE_FAST), 
        new ExtendClimbCommand(climber, SimpleExtendMovement.HOLD_RELEASE)
      ),
      new ExtendClimbCommand(climber, SimpleExtendMovement.RELEASE_TO_BOTTOM),
      new AngleClimbToPosition(climber, MID_ANGLE_ENCODER, RUN_TO_BAR_SPEED).withTimeout(ANGLE_HOOKS_TO_BAR_TIMEOUT),
      deadline(
        sequence(
          new WaitCommand(LOWER_TO_BAR_DELAY),
          new ExtendClimbCommand(climber, SimpleExtendMovement.BOTTOM_TO_EVEN)
        ),
        new AngleClimbToPosition(climber, MID_ANGLE_ENCODER, RUN_TO_BAR_SPEED, RUN_TO_BAR_TOLERANCE, RUN_TO_ANGLE_BAR_END)
      )    
    );
  }
}
