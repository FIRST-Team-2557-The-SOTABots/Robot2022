// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.Control.*;
import frc.robot.subsystems.SwerveDrive;
import static frc.robot.util.Logitech.Ports.*;
import frc.robot.util.Logitech;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private SwerveDrive swerveDrive = new SwerveDrive();

  // Driver controller and associated buttons
  private Logitech dStick = new Logitech(Driver.PORT);
  private JoystickButton da = new JoystickButton(dStick, A);
  private JoystickButton db = new JoystickButton(dStick, B);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    dStick.setDeadband(LEFT_STICK_X, Driver.LEFT_X_DEADBAND);
    dStick.setDeadband(LEFT_STICK_Y, Driver.LEFT_Y_DEADBAND);
    dStick.setDeadband(RIGHT_STICK_X, Driver.RIGHT_X_DEADBAND);

    swerveDrive.setDefaultCommand(
      new RunCommand(
        () -> {
          swerveDrive.drive(
            dStick.getRawAxis(LEFT_STICK_Y),
            dStick.getRawAxis(LEFT_STICK_X),
            dStick.getRawAxis(RIGHT_STICK_X)
          );
        },
        swerveDrive
      )
    );

    // Configure the button bindings
    configureButtonBindings();
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    da.whenPressed(new InstantCommand(
      () -> {
        swerveDrive.shiftUp();
      },
      swerveDrive
    ));

    db.whenPressed(new InstantCommand(
      () -> {
        swerveDrive.shiftDown();
      },
      swerveDrive
    ));
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
