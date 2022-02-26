// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.Logitech.Ports.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Control.Driver;
import frc.robot.Constants.Control.Manipulator;
import frc.robot.commands.RunDelivery;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.Logitech;
import frc.robot.util.UninterruptibleProxyScheduleCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private SwerveDrive swerveDrive = new SwerveDrive();
  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private Delivery delivery = new Delivery();

  // Driver controller and associated buttons
  private Logitech dStick = new Logitech(Driver.PORT);
  private JoystickButton da = new JoystickButton(dStick, A);
  private JoystickButton db = new JoystickButton(dStick, B);
  private JoystickButton dstart = new JoystickButton(dStick, START);

  // Manipulator controller and associated buttons
  private Logitech mStick = new Logitech(Manipulator.PORT);
  private JoystickButton ma = new JoystickButton(mStick, A);
  private JoystickButton mb = new JoystickButton(mStick, B);
  private JoystickButton mx = new JoystickButton(mStick, X);
  private JoystickButton my = new JoystickButton(mStick, Y);
  private JoystickButton mLB = new JoystickButton(mStick, LEFT_BUMPER);
  private JoystickButton mRB = new JoystickButton(mStick, RIGHT_BUMPER);
  // private JoystickButton mLSB = new JoystickButton(mStick, LEFT_STICK_BUTTON);
  // private JoystickButton mRSB = new JoystickButton(mStick, RIGHT_STICK_BUTTON);

  private DoubleSolenoid climbLock = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3); // TODO remove

  static double flywheelSpeed = 1000;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    climbLock.set(Value.kReverse); // TODO remove

    dStick.setDeadband(LEFT_STICK_X, Driver.LEFT_X_DEADBAND);
    dStick.setDeadband(LEFT_STICK_Y, Driver.LEFT_Y_DEADBAND);
    dStick.setDeadband(RIGHT_STICK_X, Driver.RIGHT_X_DEADBAND);
    dStick.setDeadband(LEFT_TRIGGER, Driver.LEFT_TRIGGER_DEADBAND);
    dStick.setDeadband(RIGHT_TRIGGER, Driver.RIGHT_TRIGGER_DEADBAND);

    swerveDrive.setDefaultCommand(
      // new ParallelRaceGroup(
      //   new SequentialCommandGroup(
      //     new WaitUntilCommand(
      //       () -> {
      //         if (dStick.getRawAxis(LEFT_TRIGGER) != 0.0) {
      //           swerveDrive.shiftDown();
      //         } else if (dStick.getRawAxis(RIGHT_TRIGGER) != 0.0) {
      //           swerveDrive.shiftUp();
      //         }

      //         return swerveDrive.autoShift(dStick.getRawAxis(LEFT_STICK_Y), dStick.getRawAxis(LEFT_STICK_X));
      //       }
      //     ),
      //     new WaitCommand(Constants.Swerve.SHIFT_COOLDOWN)
      //   ),
        new RunCommand(
          () -> {
            SmartDashboard.putNumber("flywheel Speed", flywheelSpeed);
            // get inputs then square them, preserving sign
            double fwd = dStick.getRawAxis(LEFT_STICK_Y);
            double str = dStick.getRawAxis(LEFT_STICK_X);
            double rot = dStick.getRawAxis(RIGHT_STICK_X);

            // pass inputs into drivetrain
            swerveDrive.drive(
              -Math.signum(fwd) * fwd * fwd * Constants.Swerve.MAX_WHEEL_SPEED,
              -Math.signum(str) * str * str * Constants.Swerve.MAX_WHEEL_SPEED,
              -Math.signum(rot) * rot * rot * Constants.Swerve.MAX_ANGULAR_SPEED
            );
            if (dStick.getRawAxis(LEFT_TRIGGER) != 0.0) {
              swerveDrive.shiftDown();
            } else if (dStick.getRawAxis(RIGHT_TRIGGER) != 0.0) {
              swerveDrive.shiftUp();
            }
          },
          swerveDrive
        )
      // )
    );

    delivery.setDefaultCommand(
      new SequentialCommandGroup(
        new WaitUntilCommand(() -> delivery.getSensor1()),
        new ParallelCommandGroup(
          new RunDelivery(delivery).withTimeout(Constants.Delivery.MAX_DELIVERY_DURATION),
          new UninterruptibleProxyScheduleCommand(
            new RunCommand(
              () -> {
                intake.retract();
                intake.run(0.0);
              }, 
              intake
            ).withTimeout(Constants.Delivery.RETRACTED_DURATION))
        ),
        new WaitCommand(Constants.Delivery.COOLDOWN)
      )
    );

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

    ma.whileHeld(
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

    mx.whenHeld(
      new RunCommand(
        () -> {
          shooter.hoodDown();
          // shooter.setMotorRPM(Constants.Shooter.UPPER_HUB_RPM); // TODO reeenable
          shooter.setMotorRPM(flywheelSpeed);
          if (shooter.atSetRPM())
            delivery.runMotor(Constants.Delivery.SHOOTING_SPEED);
          else
            delivery.runMotor(0.0);
        },
        shooter, delivery
      )
    ).whenReleased(
      new InstantCommand(
        () -> {
          shooter.runFlywheel(0.0);
          delivery.runMotor(0.0);
        }
      )
    );

    my.whenHeld(
      new RunCommand(
        () -> {
          shooter.hoodUp();
          // shooter.setMotorRPM(Constants.Shooter.LOWER_HUB_RPM);  // TODO reenable
          shooter.setMotorRPM(flywheelSpeed);
          if (shooter.atSetRPM())
            delivery.runMotor(Constants.Delivery.SHOOTING_SPEED);
          else
            delivery.runMotor(0.0);
        },
        shooter, delivery
      )
    ).whenReleased(
      new InstantCommand(
        () -> {
          shooter.runFlywheel(0.0);
          delivery.runMotor(0.0);
        }
      )
    );

    mLB.whenPressed(new InstantCommand(() -> flywheelSpeed -= 100));
    mRB.whenPressed(new InstantCommand(() -> flywheelSpeed += 100));
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
