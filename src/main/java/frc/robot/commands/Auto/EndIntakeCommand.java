// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EndIntakeCommand extends InstantCommand {
  public EndIntakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
      () -> {
        intake.retract();
        intake.run(0.0);
      }, 
      intake
    );
  }

}
