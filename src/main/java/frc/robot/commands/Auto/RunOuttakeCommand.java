// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.Delivery.*;

public class RunOuttakeCommand extends ProxyScheduleCommand {
  /** Creates a new RunOuttakeCommand. */
  public RunOuttakeCommand(Intake intake, Delivery delivery) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
      new RunCommand(
        () -> {
          intake.extend();
          intake.run(-SPEED);
          delivery.runMotor(-SHOOTING_SPEED);
        }, 
        
        intake, delivery
      ).asProxy()
    );
  }
  
}
