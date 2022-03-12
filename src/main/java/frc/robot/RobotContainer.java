// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.Logitech.Ports.*;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.LimeLight.*;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.*;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AngleProfiledPIDCommand;
import frc.robot.commands.ExtendClimbToPosition;
import frc.robot.subsystems.Climber;
import frc.robot.util.Logitech;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.LimeLight;
import frc.robot.Constants.Control.Driver;
import frc.robot.Constants.Control.Manipulator;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.RotatingSwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Climber climber = new Climber();
  private SwerveDrive swerveDrive = new SwerveDrive();
  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private Delivery delivery = new Delivery();
  private Limelight limelight = new Limelight();

  // Driver controller and associated buttons
  private Logitech dStick = new Logitech(Driver.PORT);
  private JoystickButton da = new JoystickButton(dStick, A);
  private JoystickButton db = new JoystickButton(dStick, B);
  private JoystickButton dx = new JoystickButton(dStick, X);
  private JoystickButton dy = new JoystickButton(dStick, Y);
  private JoystickButton dlb = new JoystickButton(dStick, LEFT_BUMPER);
  private JoystickButton dstart = new JoystickButton(dStick, START);

  private static double RPM = 4000;

  // Manipulator controller and associated buttons
  private Logitech mStick = new Logitech(Manipulator.PORT);
  private JoystickButton ma = new JoystickButton(mStick, A);
  private JoystickButton mb = new JoystickButton(mStick, B);
  private JoystickButton mx = new JoystickButton(mStick, X);
  private JoystickButton my = new JoystickButton(mStick, Y);
  private JoystickButton mlb = new JoystickButton(mStick, LEFT_BUMPER);
  private JoystickButton mrb = new JoystickButton(mStick, RIGHT_BUMPER);
  private JoystickButton mStart = new JoystickButton(mStick, START);


  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureDefaultCommands();

    configureButtonBindings();

    configureAutonomousCommands();
  }



  private void configureDefaultCommands() {
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
      new RunCommand(
        () -> {
          if (mStick.getRawAxis(RIGHT_TRIGGER) > 0) {
            if (mb.get())
              delivery.runMotor(-Constants.Delivery.INDEXING_SPEED);
            else
              delivery.runMotor(Constants.Delivery.INDEXING_SPEED);
          } else
            delivery.runMotor(0.0);
        }, 
        delivery  
      )
    );

    // delivery.setDefaultCommand(
    //   new SequentialCommandGroup(
    //     new WaitUntilCommand(() -> delivery.getSensor1()),
    //     new ParallelCommandGroup(
    //       new RunDelivery(delivery).withTimeout(Constants.Delivery.MAX_DELIVERY_DURATION),
    //       new UninterruptibleProxyScheduleCommand(
    //         new RunCommand(
    //           () -> {
    //             intake.retract();
    //             intake.run(0.0);
    //           }, 
    //           intake
    //         ).withTimeout(Constants.Delivery.RETRACTED_DURATION)
    //       )
    //     ),
    //     new WaitCommand(Constants.Delivery.COOLDOWN)
    //   )
    // );

    climber.setDefaultCommand(
      new RunCommand(
        () -> {
          climber.extendLeftHook(-mStick.getRawAxis(LEFT_STICK_Y));
          climber.extendRightHook(-mStick.getRawAxis(RIGHT_STICK_Y));
        }, 
        climber
      )
    );

    intake.setDefaultCommand(
      new RunCommand(
        () -> {
          if (mStick.getRawAxis(LEFT_TRIGGER) > 0) {
            intake.extend();
            intake.run(Constants.Intake.SPEED); 
          } else {
            intake.retract();
            intake.run(0.0);
          }
        },
        intake 
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

    dStick.setDeadband(LEFT_STICK_X, Driver.LEFT_X_DEADBAND);
    dStick.setDeadband(LEFT_STICK_Y, Driver.LEFT_Y_DEADBAND);
    dStick.setDeadband(RIGHT_STICK_X, Driver.RIGHT_X_DEADBAND);
    dStick.setDeadband(LEFT_TRIGGER, Driver.LEFT_TRIGGER_DEADBAND);
    dStick.setDeadband(RIGHT_TRIGGER, Driver.RIGHT_TRIGGER_DEADBAND);

    mStick.setDeadband(LEFT_TRIGGER, Manipulator.LEFT_TRIGGER_DEADBAND);
    mStick.setDeadband(RIGHT_TRIGGER, Manipulator.RIGHT_TRIGGER_DEADBAND);
    mStick.setDeadband(LEFT_STICK_Y, Manipulator.LEFT_STICK_Y_DEADBAND);
    mStick.setDeadband(RIGHT_STICK_Y, Manipulator.RIGHT_STICK_Y_DEADBAND);
    
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

    mStart.whileHeld(
      new InstantCommand(
        () -> {
          climber.retractHooksNoEncoderLimit();
        },
        climber
      )
    ).whenReleased(
      new InstantCommand(
        () -> {
          climber.extendLeftHook(0);
          climber.extendRightHook(0);
        },
        climber
      )
    );

    mx.whenHeld(
      new ConditionalCommand(
        parallel(
          new PIDCommand(
            new PIDController(TARGET_SEARCH_KP, TARGET_SEARCH_KI, TARGET_SEARCH_KD), 
            () -> limelight.getX(), 
            LIMELIGHT_CENTER, 
            (double output) -> {
              // get inputs then square them, preserving sign
              double fwd = dStick.getRawAxis(LEFT_STICK_Y);
              double str = dStick.getRawAxis(LEFT_STICK_X);
              if (Math.abs(LIMELIGHT_CENTER - limelight.getX()) < Constants.LimeLight.AUTOAIM_TOLERANCE) 
                output = 0;

              // pass inputs into drivetrain
              swerveDrive.drive(
                  -Math.signum(fwd) * fwd * fwd * Constants.Swerve.MAX_WHEEL_SPEED,
                  -Math.signum(str) * str * str * Constants.Swerve.MAX_WHEEL_SPEED,
                  output
              );

              if (dStick.getRawAxis(LEFT_TRIGGER) != 0.0) {
                swerveDrive.shiftDown();
              } else if (dStick.getRawAxis(RIGHT_TRIGGER) != 0.0) {
                swerveDrive.shiftUp();
              }
            },
            swerveDrive
          ),
          new RunCommand(
            () -> {
              shooter.hoodUp();
              shooter.setMotorRPM(shooter.calculateRPM(limelight.getY()));

              if (shooter.readyToShoot())
                delivery.runMotor(Constants.Delivery.SHOOTING_SPEED);
              else
                delivery.runMotor(0.0);
            }, 
            shooter, delivery
          )
        ),
        new RunCommand(
          () -> {
            shooter.hoodDown();
            shooter.setMotorRPM(Constants.Shooter.UPPER_HUB_RPM);
            if (shooter.readyToShoot())
              delivery.runMotor(Constants.Delivery.SHOOTING_SPEED);
            else
              delivery.runMotor(0.0);
          },
          shooter, delivery
        ), 
        () -> limelight.targetDetected()
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
          shooter.setMotorRPM(Constants.Shooter.LOWER_HUB_RPM);
          if (shooter.readyToShoot())
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
        },
        shooter, delivery
      )
    );

    mlb.whenPressed(
      sequence(
        new InstantCommand(() -> shooter.hoodDown()),
        new InstantCommand(() -> climber.unlock()),
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
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.MID_TO_TOP, climber).withTimeout(Constants.Climber.ANGLED_EXTEND_TIMEOUT),
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
          ).withTimeout(Constants.Climber.TIMED_ANGLE_DURATION_2).andThen(() -> climber.runAngle(0)),
          new ExtendClimbToPosition(Constants.Climber.ExtendMovement.HANG_BOTTOM, climber)
        ),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.BOTTOM_TO_EVEN, climber),
        new WaitUntilCommand(() -> {
          SmartDashboard.putString("waiting", "step 2");
          return mrb.get();}),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.EVEN_TO_MID, climber),
        climber.generateAnglePIDCommand(Constants.Climber.AngleMovement.MID_TO_MAX),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.MID_TO_TOP, climber).withTimeout(Constants.Climber.ANGLED_EXTEND_TIMEOUT),
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
          ).withTimeout(Constants.Climber.TIMED_ANGLE_DURATION_2).andThen(() -> climber.runAngle(0)),
          new ExtendClimbToPosition(Constants.Climber.ExtendMovement.HANG_BOTTOM, climber)
        ),
        new ExtendClimbToPosition(Constants.Climber.ExtendMovement.BOTTOM_TO_EVEN, climber)
      )     
    );
  }

  public void configureAutonomousCommands() {
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);

    autoChooser.addOption("Back up",
      sequence(
        new InstantCommand(() -> swerveDrive.setFieldCentricActive(false)),
        new RunCommand(
          () -> swerveDrive.drive(-1, 0, 0.0),
          swerveDrive
        ).withTimeout(Constants.Auto.BACK_UP_AUTO_DURATION)
      )
    );

    autoChooser.addOption("Shoot high Back up",
      sequence(
        new InstantCommand(() -> swerveDrive.setFieldCentricActive(false)),
        new RunCommand(
          () -> {
            shooter.hoodDown();
            shooter.setMotorRPM(Constants.Shooter.UPPER_HUB_RPM);
            if (shooter.readyToShoot())
              delivery.runMotor(Constants.Delivery.SHOOTING_SPEED);
            else
              delivery.runMotor(0.0);
          },
          shooter, delivery
        ).withTimeout(Constants.Auto.SHOOT_HIGH_BACK_SHOOT_DURATION),
        new InstantCommand(
          () -> {
            shooter.setMotorRPM(0.0);
            delivery.runMotor(0.0);
          },
          shooter, delivery
        ),
        new InstantCommand(
          () -> swerveDrive.setFieldCentricActive(false)
        ),
        new RunCommand(
          () -> swerveDrive.drive(-1, 0, 0.0),
          swerveDrive
        ).withTimeout(Constants.Auto.SHOOT_HIGH_BACK_DRIVE_DURATION)
      )
    );

    // autoChooser.addOption("2 ball general",
    //   sequence (
    //     new InstantCommand(() -> swerveDrive.setFieldCentricActive(false)),
    //     new ParallelDeadlineGroup(
    //       new RunCommand(() -> swerveDrive.drive(-1, 0, 0.0), swerveDrive).withTimeout(Constants.Auto.BACK_UP_AUTO_DURATION),
    //       sequence(
    //         new WaitCommand(0.0),
    //         new InstantCommand(
    //           () -> {
    //             intake.extend();
    //             intake.run(Constants.Intake.SPEED);
    //           },
    //           intake
    //         )
    //       ),
    //       new InstantCommand(
    //         () -> {
    //           intake.retract();
    //           intake.run(0.0);
    //         },
    //         intake
    //       )
    //     ),
    //     new RunCommand(() -> swerveDrive.drive(0.0, 0, 0), swerveDrive).withTimeout(0.0)
    //   )
    // );

    PathPlannerTrajectory path1A = PathPlanner.loadPath("Path_1_A", Constants.Auto.MAX_WHEEL_SPEED, Constants.Auto.MAX_WHEEL_ACCELERATION);
    PathPlannerTrajectory path1B = PathPlanner.loadPath("Path_1_B", Constants.Auto.MAX_WHEEL_SPEED, Constants.Auto.MAX_WHEEL_ACCELERATION);
    PathPlannerTrajectory path1C = PathPlanner.loadPath("Path_1_C", Constants.Auto.MAX_WHEEL_SPEED, Constants.Auto.MAX_WHEEL_ACCELERATION);
    PathPlannerTrajectory path1D = PathPlanner.loadPath("Path_1_D", Constants.Auto.MAX_WHEEL_SPEED, Constants.Auto.MAX_WHEEL_ACCELERATION);

    autoChooser.addOption("Auto 1",
      sequence(
        new InstantCommand(
          () -> {
            swerveDrive.shiftUp();
            swerveDrive.setPose(path1A.getInitialState());
          }, 
          swerveDrive
        ),
        swerveDrive.generatePPSwerveControllerCommand(path1A),
        new InstantCommand(() -> swerveDrive.drive(0, 0, 0)),
        new RunCommand(
          () -> {
            shooter.hoodUp();
            shooter.setMotorRPM(Constants.Shooter.LOWER_HUB_RPM);
            if (shooter.readyToShoot())
              delivery.runMotor(Constants.Delivery.SHOOTING_SPEED);
            else
              delivery.runMotor(0.0);
          },
          shooter, delivery
        ).withTimeout(Constants.Auto.PATH_1_SHOOT_1_DURATION),
        new InstantCommand(
          () -> {
            shooter.runFlywheel(0.0);
            delivery.runMotor(0.0);
          },
          shooter, delivery
        ),
        race(
          swerveDrive.generatePPSwerveControllerCommand(path1B),
          sequence(
            new WaitCommand(path1A.getTotalTimeSeconds() - Constants.Auto.INTAKE_EXTENSION_TIME),
            new RunCommand(
              () -> {
                intake.extend();
                intake.run(Constants.Intake.SPEED);
              }, 
              intake
            )
          )
        ),
        new InstantCommand(
          () -> {
            swerveDrive.drive(0, 0, 0);
            intake.retract();
            intake.run(0.0);
          }
        ),
        new RunCommand(
          () -> {
            shooter.hoodUp();
            shooter.setMotorRPM(Constants.Shooter.LOWER_HUB_RPM);
            if (shooter.readyToShoot())
              delivery.runMotor(Constants.Delivery.SHOOTING_SPEED);
            else
              delivery.runMotor(0.0);
          },
          shooter, delivery
        ).withTimeout(Constants.Auto.PATH_1_SHOOT_2_DURATION),
        new InstantCommand(
          () -> {
            shooter.runFlywheel(0.0);
            delivery.runMotor(0.0);
          },
          shooter, delivery
        ),
        race(
          swerveDrive.generatePPSwerveControllerCommand(path1C),
          new RunCommand(
            () -> {
              intake.extend();
              intake.run(Constants.Intake.SPEED);
            }, 
            intake
          )
        ),
        new InstantCommand(
          () -> {
            swerveDrive.drive(0, 0, 0);
            intake.retract();
            intake.run(0.0);
          }
        ),
        new RunCommand(
          () -> {
            shooter.hoodUp();
            shooter.setMotorRPM(Constants.Shooter.LOWER_HUB_RPM);
            if (shooter.readyToShoot())
              delivery.runMotor(Constants.Delivery.SHOOTING_SPEED);
            else
              delivery.runMotor(0.0);
          },
          shooter, delivery
        ).withTimeout(Constants.Auto.PATH_1_SHOOT_3_DURATION),
        new InstantCommand(
          () -> {
            shooter.runFlywheel(0.0);
            delivery.runMotor(0.0);
          },
          shooter, delivery
        )
      )
    );

    // autoChooser.addOption("3 ball",
    //   new RotatingSwerveControllerCommand(
    //     TrajectoryGenerator.generateTrajectory(
    //       new Pose2d(0, 0, new Rotation2d(0.0)),
    //       List.of(
    //         new Translation2d(1, 0),
    //         new Translation2d(2, 0)
    //       ),
    //       new Pose2d(3, 0, new Rotation2d(0.0)),
    //       new TrajectoryConfig(
    //         Constants.Auto.MAX_WHEEL_SPEED, 
    //         Constants.Auto.MAX_WHEEL_ACCELERATION
    //       ).setKinematics(swerveDrive.getKinematics())
    //     ), 
    //     () -> swerveDrive.getSwervePose(), 
    //     swerveDrive.getKinematics(), 
    //     new PIDController(Constants.Auto.X_PID_KP, 0.0, 0.0), 
    //     new PIDController(Constants.Auto.Y_PID_KP, 0.0, 0.0), 
    //     new ProfiledPIDController(
    //       Constants.Auto.ANGLE_PID_KP, 0.0, 0.0, 
    //       new TrapezoidProfile.Constraints(Constants.Auto.MAX_ANGULAR_SPEED, Constants.Auto.MAX_ANGULAR_ACCELERATION)
    //     ),
    //     (SwerveModuleState[] states) -> swerveDrive.drive(states), 
    //     swerveDrive
    //   )
    // );
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SendableChooser<Command> getAutonomousChooser() {
    // An ExampleCommand will run in autonomous
    return autoChooser;
  }
}
