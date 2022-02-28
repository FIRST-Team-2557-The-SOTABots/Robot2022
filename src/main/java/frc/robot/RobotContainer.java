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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.TOP_TO_BOTTOM, climber)
        // new RunCommand(
        //   () -> {
        //     climber.runAngle(Constants.Climber.TIMED_ANGLE_SPEED);
        //     SmartDashboard.putNumber("Running", Timer.getFPGATimestamp());
        //   }, 
        //   climber
        // ).withTimeout(Constants.Climber.TIMED_ANGLE_DURATION).andThen(() -> climber.runAngle(0))
        // new WaitUntilCommand(() -> {
        //   SmartDashboard.putString("waiting", "step 2");
        //   return mrb.get();}),
        // new ExtendClimbToPosition(Constants.Climber.ExtendMovement.BOTTOM_TO_MID, climber)
        // new ExtendClimbToPosition(Constants.Climber.MIN_EXTEND_ENCODER, climber),
        // climber.generateAnglePIDCommand(Constants.Climber.AngleMovement.MIN_TO_MID)
        // new ExtendClimbToPosition(Constants.Climber.MID_EXTEND_ENCODER, climber),
        // new WaitUntilCommand(mrb::get),
        // climber.generateAnglePIDCommand(Constants.Climber.MAX_ANGLE_ENCODER),
        // new ExtendClimbToPosition(Constants.Climber.MAX_EXTEND_ENCODER, climber),
        // climber.generateAnglePIDCommand(Constants.Climber.HIGH_ANGLE_ENCODER),
        // new WaitUntilCommand(mrb::get),
        // parallel(
        //   new ExtendClimbToPosition(Constants.Climber.MIN_EXTEND_ENCODER, climber),
        //   climber.generateAnglePIDCommand(Constants.Climber.MAX_ANGLE_ENCODER)
        // ),
        // new WaitUntilCommand(mrb::get),
        // climber.generateAnglePIDCommand(Constants.Climber.HIGH_ANGLE_ENCODER),
        // new ExtendClimbToPosition(Constants.Climber.MID_EXTEND_ENCODER, climber),
        // climber.generateAnglePIDCommand(Constants.Climber.MIN_ANGLE_ENCODER),
        // new ExtendClimbToPosition(Constants.Climber.MIN_EXTEND_ENCODER, climber),
        // climber.generateAnglePIDCommand(Constants.Climber.MID_ANGLE_ENCODER),
        // new ExtendClimbToPosition(Constants.Climber.MID_EXTEND_ENCODER, climber)
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
