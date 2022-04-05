// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Intake;
import frc.robot.util.UnendingProxyScheduleCommand;

import static frc.robot.Constants.Intake.*;

public class RunIntakeCommand extends UnendingProxyScheduleCommand {
  /** Creates a new RunIntakeCommand. */
  public RunIntakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
      new RunCommand(
        () -> {
          intake.extend();
          intake.run(SPEED);
        }, 
        intake
      )
    );
  }
  
}
