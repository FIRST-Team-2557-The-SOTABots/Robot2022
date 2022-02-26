// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.Climber.*;

public class ExtendClimbToPosition extends CommandBase {
  private ProfiledPIDController leftController;
  private ProfiledPIDController rightController;
  private Climber climber;

  /** Creates a new ExtendClimbToPosition. */
  public ExtendClimbToPosition(double goal, Climber climber) {
    
    leftController = new ProfiledPIDController(
      EXTEND_PID_KP, EXTEND_PID_KI, EXTEND_PID_KD, 
      new TrapezoidProfile.Constraints(EXTEND_MAX_VELOCITY, EXTEND_MAX_ACCELERATION)
    );
    leftController.setGoal(goal);
    rightController = new ProfiledPIDController(
      EXTEND_PID_KP, EXTEND_PID_KI, EXTEND_PID_KD, 
      new TrapezoidProfile.Constraints(EXTEND_MAX_VELOCITY, EXTEND_MAX_ACCELERATION)
    );
    rightController.setGoal(goal);
    this.climber = climber;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.extendLeftHook(
      leftController.calculate(
        climber.getLeftEncoderPosition()
      )
    );

    climber.extendRightHook(
      rightController.calculate(
        climber.getRightEncoderPosition()
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.extendLeftHook(0.0);
    climber.extendRightHook(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftController.atGoal() && rightController.atGoal();
  }
}
