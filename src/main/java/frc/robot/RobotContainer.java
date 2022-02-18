// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.Control.*;

import frc.robot.Constants;
import frc.robot.commands.RunDelivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
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

  public static SwerveDrive swerveDrive = new SwerveDrive();
  public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  public static Sensors sensors = new Sensors();

  // Driver controller and associated buttons
  private Logitech dStick = new Logitech(Driver.PORT);
  private JoystickButton da = new JoystickButton(dStick, A);
  private JoystickButton db = new JoystickButton(dStick, B);
  private JoystickButton dx = new JoystickButton(dStick, X);
  private JoystickButton dy = new JoystickButton(dStick, Y);
  private JoystickButton dstart = new JoystickButton(dStick, START);

  // Manipulator controller and associated buttons
  private Logitech mStick = new Logitech(Manipulator.PORT);
  private JoystickButton ma = new JoystickButton(mStick, A);
  private JoystickButton mb = new JoystickButton(mStick, B);
  private JoystickButton mx = new JoystickButton(mStick, X);
  private JoystickButton my = new JoystickButton(mStick, Y);
  private JoystickButton mstart = new JoystickButton(mStick, START);
  private JoystickButton mrt = new JoystickButton(mStick, RIGHT_TRIGGER);

  public static boolean isBall = false;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    dStick.setDeadband(LEFT_STICK_X, Driver.LEFT_X_DEADBAND);
    dStick.setDeadband(LEFT_STICK_Y, Driver.LEFT_Y_DEADBAND);
    dStick.setDeadband(RIGHT_STICK_X, Driver.RIGHT_X_DEADBAND);
    dStick.setDeadband(LEFT_TRIGGER, Driver.LEFT_TRIGGER_DEADBAND);
    dStick.setDeadband(RIGHT_TRIGGER, Driver.RIGHT_TRIGGER_DEADBAND);

    swerveDrive.setDefaultCommand(
      new RunCommand(
        () -> {
          double y = dStick.getRawAxis(LEFT_STICK_Y);
          double x = dStick.getRawAxis(LEFT_STICK_X);
          double w = dStick.getRawAxis(RIGHT_STICK_X);

          y = y < 0 ? -y * y : y * y;
          x = x < 0 ? -x * x : x * x;
          w = w < 0 ? -w * w : w * w;


          swerveDrive.drive(
            -y * Constants.Swerve.MAX_WHEEL_SPEED,
            -x * Constants.Swerve.MAX_WHEEL_SPEED,
            -w * Constants.Swerve.MAX_ANGULAR_SPEED
          );
          if (dStick.getRawAxis(LEFT_TRIGGER) != 0.0)
            swerveDrive.shiftDown();
          else if (dStick.getRawAxis(RIGHT_TRIGGER) != 0.0)
            swerveDrive.shiftUp();
        },
        swerveDrive
      )
    );

    sensors.setDefaultCommand(
      new ConditionalCommand(onTrue, new RunDelivery(), isBall)
      
    );

    shooter.setDefaultCommand(
      new RunCommand(
        () -> {

          shooter.runFlywheel(mStick.getRawAxis(RIGHT_TRIGGER));
          SmartDashboard.putNumber("flywheel speed", shooter.getSpeed());

        }, shooter)
    );

    // shooter.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       if(shooter.getSpeed() != 0 && shooter.getSpeed() >= ){

    //       }

    //     }, shooter)
    // );

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
    da.whenPressed(
      new InstantCommand(
        () -> {
          swerveDrive.setFieldCentricActive(true);
        },
        swerveDrive
      )
    );

    db.whenPressed(
      new InstantCommand(
        () -> {
          swerveDrive.setFieldCentricActive(false);
        },
        swerveDrive
      )
    );

    dstart.whenPressed(
      new InstantCommand(
        () -> {
          swerveDrive.resetGyro();
        },
        swerveDrive
      )
    );

    ma.whenHeld(
      new InstantCommand(
        () -> {
          intake.extend();
          intake.run(Constants.Intake.SPEED);
        },
        intake
      )
    ).whenReleased(
      new InstantCommand(
        () -> {
          intake.run(0.0);
          intake.retract();
        }
      )
    );
    
    // mrt.whenHeld(
    //   new InstantCommand(
    //     () -> {
    //       shooter.runFlywheel(mStick.getRawAxis(RIGHT_TRIGGER));
    //       SmartDashboard.putNumber("flywheel speed", shooter.getSpeed());
    //     },
    //     shooter
    //   )
    // ).whenReleased(
    //   new InstantCommand(
    //     () -> {
    //       shooter.runFlywheel(0);
    //     }
    //   )
    // );
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
