// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Auto.*;

public class RevFlywheelCommand extends RunCommand {
  /** Creates a new RevFlywheelCommand. */
  public RevFlywheelCommand(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
      () -> {
        shooter.setMotorRPM(FLYWHEEL_IDLE_SPEED);
      }, 
      shooter
    );
  }

}
