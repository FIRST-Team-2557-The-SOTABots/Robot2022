// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Climber.AngleMovement;
import frc.robot.commands.AngleProfiledPIDCommand;
import frc.robot.commands.ExtendClimbToPosition;
import frc.robot.subsystems.Climber;
import frc.robot.util.Logitech;
import static frc.robot.util.Logitech.Ports.*;
import static edu.wpi.first.wpilibj2.command.CommandGroupBase.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Logitech mStick = new Logitech(1);
  private JoystickButton mlb = new JoystickButton(mStick, LEFT_BUMPER);
  private JoystickButton mrb = new JoystickButton(mStick, RIGHT_BUMPER);
  private JoystickButton mStart = new JoystickButton(mStick, START);

  private Climber climber = new Climber();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    climber.setDefaultCommand(
      new RunCommand(
        () -> {
          climber.extendLeftHook(-mStick.getRawAxis(LEFT_STICK_Y));
          climber.extendRightHook(-mStick.getRawAxis(RIGHT_STICK_Y));
          // climber.runAngle(mStick.getRawAxis(RIGHT_STICK_X));
        }, 
        climber
      )
    );

    mlb.whenPressed(
      sequence(
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.BOTTOM_TO_TOP, climber),
        new WaitUntilCommand(() -> {
          SmartDashboard.putString("waiting", "step 1");
          return mrb.get();}),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.TOP_TO_BOTTOM, climber),
        new ParallelRaceGroup(
          new RunCommand(
            () -> {
              climber.runAngle(Constants.Climber.TIMED_ANGLE_SPEED);
            }
          ).withTimeout(Constants.Climber.TIMED_ANGLE_DURATION).andThen(() -> climber.runAngle(0)),
          new ExtendClimbToPosition(Constants.Climber.ExtendMovement.HANG_BOTTOM, climber)
        ),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.BOTTOM_TO_EVEN, climber),
        new WaitUntilCommand(() -> {
          SmartDashboard.putString("waiting", "step 2");
          return mrb.get();}),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.EVEN_TO_MID, climber),
        climber.generateAnglePIDCommand(Constants.Climber.AngleMovement.MID_TO_MAX),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.MID_TO_TOP, climber),
        new AngleProfiledPIDCommand(Constants.Climber.AngleMovement.MAX_TO_HIGH, climber),
        new ParallelRaceGroup(
          climber.generateAnglePIDCommand(Constants.Climber.AngleMovement.HOLD_HIGH),
          sequence(
            new WaitCommand(Constants.Climber.ANGLE_PID_PAUSE),
            new ExtendClimbToPosition(Constants.Climber.ExtendMovement.TOP_TO_HIGH, climber)
          )
        ),
        new WaitUntilCommand(() -> {
          SmartDashboard.putString("waiting", "step 3");
          return mrb.get();}),
        new ParallelRaceGroup(
          new RunCommand(
            () -> climber.runAngle(Constants.Climber.TIMED_ANGLE_SPEED)
          ).andThen(new InstantCommand(() -> climber.runAngle(0.0))),
          new ExtendClimbToPosition(Constants.Climber.ExtendMovement.HIGH_TO_BOTTOM, climber)
        ),
        parallel(
          climber.generateAnglePIDCommand(Constants.Climber.AngleMovement.MAX_TO_HIGH_NO_LOAD),
          new ExtendClimbToPosition(Constants.Climber.ExtendMovement.BOTTOM_TO_MID, climber)
        ),
        parallel(
          climber.generateAnglePIDCommand(Constants.Climber.AngleMovement.HIGH_TO_MIN),
          new ExtendClimbToPosition(Constants.Climber.ExtendMovement.MID_TO_BOTTOM, climber)
        ),
        new ParallelRaceGroup(
          new RunCommand(
            () -> {
              climber.runAngle(Constants.Climber.TIMED_ANGLE_SPEED);
            }
          ).withTimeout(Constants.Climber.TIMED_ANGLE_DURATION).andThen(() -> climber.runAngle(0)),
          new ExtendClimbToPosition(Constants.Climber.ExtendMovement.HANG_BOTTOM, climber)
        ),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.BOTTOM_TO_EVEN, climber),
        
        new WaitUntilCommand(() -> {
          SmartDashboard.putString("waiting", "step 2");
          return mrb.get();}),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.EVEN_TO_MID, climber),
        climber.generateAnglePIDCommand(Constants.Climber.AngleMovement.MID_TO_MAX),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.MID_TO_TOP, climber),
        new AngleProfiledPIDCommand(Constants.Climber.AngleMovement.MAX_TO_HIGH, climber),
        new ParallelRaceGroup(
          climber.generateAnglePIDCommand(Constants.Climber.AngleMovement.HOLD_HIGH),
          sequence(
            new WaitCommand(Constants.Climber.ANGLE_PID_PAUSE),
            new ExtendClimbToPosition(Constants.Climber.ExtendMovement.TOP_TO_HIGH, climber)
          )
        ),
        new WaitUntilCommand(() -> {
          SmartDashboard.putString("waiting", "step 3");
          return mrb.get();}),
        new ParallelRaceGroup(
          new RunCommand(
            () -> climber.runAngle(Constants.Climber.TIMED_ANGLE_SPEED)
          ).andThen(new InstantCommand(() -> climber.runAngle(0.0))),
          new ExtendClimbToPosition(Constants.Climber.ExtendMovement.HIGH_TO_BOTTOM, climber)
        ),
        parallel(
          climber.generateAnglePIDCommand(Constants.Climber.AngleMovement.MAX_TO_HIGH_NO_LOAD),
          new ExtendClimbToPosition(Constants.Climber.ExtendMovement.BOTTOM_TO_MID, climber)
        ),
        parallel(
          climber.generateAnglePIDCommand(Constants.Climber.AngleMovement.HIGH_TO_MIN),
          new ExtendClimbToPosition(Constants.Climber.ExtendMovement.MID_TO_BOTTOM, climber)
        ),
        new ParallelRaceGroup(
          new RunCommand(
            () -> {
              climber.runAngle(Constants.Climber.TIMED_ANGLE_SPEED);
            }
          ).withTimeout(Constants.Climber.TIMED_ANGLE_DURATION).andThen(() -> climber.runAngle(0)),
          new ExtendClimbToPosition(Constants.Climber.ExtendMovement.HANG_BOTTOM, climber)
        ),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.BOTTOM_TO_EVEN, climber)
      )     
    );
  }



  public void resetRobot() {
    climber.reset();
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    mStart.whenPressed(
      new RunCommand(
        () -> climber.retractHooksNoEncoderLimit(),
        climber
      )
    ).whenReleased(
      new InstantCommand(
        () -> {
          climber.extendLeftHook(0.0);
          climber.extendRightHook(0.0);
        }, 
        climber  
      )
    );
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
