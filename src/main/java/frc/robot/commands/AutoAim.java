// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import static frc.robot.util.Logitech.Ports.*;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Delivery.*;
import static frc.robot.Constants.LimeLight.*;

public class AutoAim extends CommandBase {

  private PIDController controller;
  private Limelight limelight;
  private Shooter shooter;
  private Delivery delivery;
  private SwerveDrive swerveDrive;
  private Joystick dStick;

  /** Creates a new AutoAim. */
  public AutoAim(Limelight limelight, Shooter shooter, Delivery delivery, SwerveDrive swerveDrive, Joystick dStick) {
    addRequirements(shooter, delivery, swerveDrive);

    controller = new PIDController(TARGET_SEARCH_KP, TARGET_SEARCH_KI, TARGET_SEARCH_KD);
    this.limelight = limelight;
    this.shooter = shooter;
    this.delivery = delivery;
    this.swerveDrive = swerveDrive;
    this.dStick = dStick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = controller.calculate(limelight.getX());

    double fwd = dStick.getRawAxis(LEFT_STICK_Y);
    double str = dStick.getRawAxis(LEFT_STICK_X);
    double rot = dStick.getRawAxis(RIGHT_STICK_X);

    swerveDrive.drive(
      -Math.signum(fwd) * fwd * fwd * MAX_WHEEL_SPEED,
      -Math.signum(str) * str * str * MAX_WHEEL_SPEED,
      limelight.targetDetected() ? // Me when the nested ternerary operater
      Math.abs(LIMELIGHT_CENTER - limelight.getX()) < AUTOAIM_TOLERANCE ? 
      0 : output : -Math.signum(rot) * rot * rot * MAX_ANGULAR_SPEED
    );
              
    shooter.hoodUp();

    shooter.setMotorRPM(
      RPM_EQUATION.apply(limelight.getY())
    );

    if (shooter.readyToShoot() && limelight.targetDetected()) {
      delivery.runMotor(
        SHOOTING_SPEED
      );
    } else {
      delivery.runMotor(0.0);
    }

    if (dStick.getRawAxis(LEFT_TRIGGER) != 0.0) 
      swerveDrive.shiftDown();
    else if (dStick.getRawAxis(RIGHT_TRIGGER) != 0.0) 
      swerveDrive.shiftUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.runFlywheel(0.0);
    delivery.runMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
