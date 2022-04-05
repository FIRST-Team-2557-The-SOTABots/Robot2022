// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.LimeLight.*;
import static frc.robot.Constants.Delivery.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootCommand extends ParallelCommandGroup {
  /** Creates a new AutoShootCommand. */
  public AutoShootCommand(Shooter shooter, SwerveDrive swerveDrive, Delivery delivery, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PIDCommand(
        new PIDController(TARGET_SEARCH_KP, TARGET_SEARCH_KI, TARGET_SEARCH_KD), 
        () -> limelight.getX(), 
        LIMELIGHT_CENTER, 
        (double output) -> {
          if (Math.abs(LIMELIGHT_CENTER - limelight.getX()) < AUTOAIM_TOLERANCE) 
            output = 0;

          swerveDrive.drive(0, 0, output);
        },
        swerveDrive
      ),
      new RunCommand(
        () -> {
          shooter.hoodUp();
          shooter.setMotorRPM(shooter.calculateRPM(limelight.getY()));

          if (shooter.readyToShoot()) {
            delivery.runMotor(SHOOTING_SPEED);
            System.out.println(true);
          }
          else {
            delivery.runMotor(0.0);
            System.out.println(false);
          }
        }, 
        shooter, delivery
      )
    );
  }
  
}
