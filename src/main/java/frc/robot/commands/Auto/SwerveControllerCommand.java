// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class SwerveControllerCommand extends PPSwerveControllerCommand {
  /** Creates a new PPSwerveControllerCommand. */
  public SwerveControllerCommand(SwerveDrive swerveDrive, PathPlannerTrajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
      trajectory,
      swerveDrive::getPose,
      swerveDrive.getKinematics(),
      new PIDController(Constants.Auto.TRANSLATE_PID_KP, 0, 0),
      new PIDController(Constants.Auto.TRANSLATE_PID_KP, 0, 0),
      new ProfiledPIDController(
        Constants.Auto.ANGLE_PID_KP, 0, 0, 
        new TrapezoidProfile.Constraints(
          Constants.Auto.MAX_ANGULAR_SPEED, Constants.Auto.MAX_ANGULAR_ACCELERATION
        )
      ), 
      swerveDrive::drive, 
      swerveDrive
    );
  }
   
}
