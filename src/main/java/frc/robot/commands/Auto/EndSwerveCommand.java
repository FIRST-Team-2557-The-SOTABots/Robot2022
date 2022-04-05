// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrive;

public class EndSwerveCommand extends InstantCommand {
  /** Creates a new EndSwerveCommand. */
  public EndSwerveCommand(SwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
      () -> {
        swerveDrive.drive(0.0, 0.0, 0.0);
      }, 
      swerveDrive
    );
  }

}
