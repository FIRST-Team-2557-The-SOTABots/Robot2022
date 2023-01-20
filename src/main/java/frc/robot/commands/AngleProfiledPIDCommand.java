// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Climber.AngleMovement;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.Climber.*;

public class AngleProfiledPIDCommand extends CommandBase {

  private Climber climber;
  private ProfiledPIDController controller;
  private AngleMovement angleMovement;

  /** Creates a new AngleProfiledPIDCommand. */
  public AngleProfiledPIDCommand(AngleMovement angleMovement, Climber climber) {
    this.climber = climber;

    controller = new ProfiledPIDController(
      angleMovement.kp, angleMovement.ki, angleMovement.kd,
      new TrapezoidProfile.Constraints(ANGLE_PID_MAX_VELOCITY, ANGLE_PID_MAX_ACCELERATION)
    );
    controller.setGoal(angleMovement.setpoint);
    controller.setTolerance(angleMovement.tolerance);
    
    this.angleMovement = angleMovement;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Angle Command", "Initialize");
    controller.reset(climber.getAngleEncoderPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Angle Command", "Executing");

    // as the robot gets closer to its setpoint in MAX_TO_HIGH movement, KP is increased to compensate for increased torque from gravity
    if (angleMovement == AngleMovement.MAX_TO_HIGH) {
      if (Math.abs(angleMovement.setpoint - climber.getAngleEncoderPosition()) < ANGLE_PID_CHANGE_KP_RANGE) {
        controller.setP(ANGLE_PID_CHANGE_KP_VALUE);
      } else {
        controller.setP(angleMovement.kp);
      }
      
      double output = controller.calculate(climber.getAngleEncoderPosition());
      climber.runAngle(output);
    }
  } 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Angle Command", "End");
    climber.runAngle(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atGoal();
  }
}
