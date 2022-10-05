// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.Climber.*;

import java.text.FieldPosition;

/**
 * A command to run the angle hooks to a specified position
 */
public class AngleClimbToPosition extends CommandBase {

  private Climber climber;
  private int setpoint;
  private double speed;
  private double tolerance;
  private boolean end;
  
  /** Creates a new AngleClimbToPosition. */
  public AngleClimbToPosition(Climber climber, int setpoint, double speed) {
    this(climber, setpoint, speed, RUN_TO_ANGLE_TOLERANCE, true);
  }

  public AngleClimbToPosition(Climber climber, int setpoint, double speed, double tolerance) {
    this(climber, setpoint, speed, tolerance, true);
  }

  public AngleClimbToPosition(Climber climber, int setpoint, double speed, double tolerance, boolean end) {
    this.climber = climber;
    this.setpoint = setpoint;
    this.speed = speed;
    this.tolerance = tolerance;
    this.end = end;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Angle Command", "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Angle Command", "Executing");
    // run forward if below setpoint, backwards otherwise
    if (Math.abs(setpoint - climber.getAngleEncoderPosition()) < tolerance) {
      climber.runAngle(0.0);
    } else {
      if (setpoint - climber.getAngleEncoderPosition() > 0) {
        climber.runAngle(speed);
      } else {
        if (end) // don't run backwards if angling to the bar
          climber.runAngle(-speed);
      }
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
    return end ? Math.abs(setpoint - climber.getAngleEncoderPosition()) < tolerance : false;
  }
}
