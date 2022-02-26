// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private Climber climber;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    climber = new Climber();

    climber.setDefaultCommand(
      new RunCommand(
        () -> {
          climber.extendLeftHook(-mStick.getRawAxis(LEFT_STICK_Y));
          climber.extendRightHook(-mStick.getRawAxis(LEFT_STICK_Y));
          climber.runAngle(-mStick.getRawAxis(RIGHT_STICK_Y));
        }, 
        climber
      )
    );

    mlb.whenPressed(
      sequence(
        new ExtendClimbToPosition(Constants.Climber.MAX_EXTEND_ENCODER, climber),
        new WaitUntilCommand(mrb::get),
        new ExtendClimbToPosition(Constants.Climber.MIN_EXTEND_ENCODER, climber),
        climber.generateAnglePIDCommand(Constants.Climber.MID_ANGLE_ENCODER),
        new ExtendClimbToPosition(Constants.Climber.MID_EXTEND_ENCODER, climber),
        new WaitUntilCommand(mrb::get),
        climber.generateAnglePIDCommand(Constants.Climber.MAX_ANGLE_ENCODER),
        new ExtendClimbToPosition(Constants.Climber.MAX_EXTEND_ENCODER, climber),
        climber.generateAnglePIDCommand(Constants.Climber.HIGH_ANGLE_ENCODER),
        new WaitUntilCommand(mrb::get),
        parallel(
          new ExtendClimbToPosition(Constants.Climber.MIN_EXTEND_ENCODER, climber),
          climber.generateAnglePIDCommand(Constants.Climber.MAX_ANGLE_ENCODER)
        ),
        new WaitUntilCommand(mrb::get),
        climber.generateAnglePIDCommand(Constants.Climber.HIGH_ANGLE_ENCODER),
        new ExtendClimbToPosition(Constants.Climber.MID_EXTEND_ENCODER, climber),
        climber.generateAnglePIDCommand(Constants.Climber.MIN_ANGLE_ENCODER),
        new ExtendClimbToPosition(Constants.Climber.MIN_EXTEND_ENCODER, climber),
        climber.generateAnglePIDCommand(Constants.Climber.MID_ANGLE_ENCODER),
        new ExtendClimbToPosition(Constants.Climber.MID_EXTEND_ENCODER, climber)
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
  private void configureButtonBindings() {}



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
