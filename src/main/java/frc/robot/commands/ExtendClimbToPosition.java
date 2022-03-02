// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.Climber.*;

public class ExtendClimbToPosition extends CommandBase {
  private ProfiledPIDController leftController;
  private ProfiledPIDController rightController;
  private Climber climber;
  private ExtendMovement movementType;

  /** Creates a new ExtendClimbToPosition. */
  public ExtendClimbToPosition(ExtendMovement movementType, Climber climber) {
    this.movementType = movementType;

    double leftGoal = movementType.leftGoal;
    double rightGoal = movementType.rightGoal;
    
    leftController = new ProfiledPIDController(
      movementType.kp, movementType.ki, movementType.kd, 
      new TrapezoidProfile.Constraints(movementType.maxVelocity, movementType.maxAcceleration)
    );
    leftController.setGoal(leftGoal);
    rightController = new ProfiledPIDController(
      movementType.kp, movementType.ki, movementType.kd,
      new TrapezoidProfile.Constraints(movementType.maxVelocity, movementType.maxAcceleration)
    );
    rightController.setGoal(rightGoal);
    this.climber = climber;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftController.reset(climber.getLeftEncoderPosition());
    rightController.reset(climber.getRightEncoderPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("left setpoint", leftController.getSetpoint().position);
    SmartDashboard.putNumber("right setpoint", rightController.getSetpoint().position);
    
    climber.extendLeftHook(
      leftController.calculate(
        climber.getLeftEncoderPosition()
      )
    );

    SmartDashboard.putNumber("left output", leftController.calculate(climber.getLeftEncoderPosition()));
    SmartDashboard.putNumber("right output", rightController.calculate(climber.getRightEncoderPosition()));

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
    boolean leftFinished = false;
    boolean rightFinished = false;
    if (movementType == ExtendMovement.MID_TO_BOTTOM || movementType == ExtendMovement.TOP_TO_BOTTOM) {
      leftFinished = climber.getLeftBotMagLimit();
      rightFinished = climber.getRightBotMagLimit();
      SmartDashboard.putString("moving to", "bottom");
    } else if (movementType == ExtendMovement.BOTTOM_TO_TOP || movementType == ExtendMovement.MID_TO_TOP) {
      leftFinished = climber.getLeftTopMagLimit();
      rightFinished = climber.getRightTopMagLimit();
      SmartDashboard.putString("moving to", "top");
    } else if (movementType == ExtendMovement.HANG_BOTTOM) {
      leftFinished = false;
      rightFinished = false;
    } else {
      leftFinished = Math.abs(climber.getLeftEncoderPosition() - leftController.getGoal().position) < EXTEND_PID_TOLERANCE;
      rightFinished = Math.abs(climber.getRightEncoderPosition() - rightController.getGoal().position) < EXTEND_PID_TOLERANCE;
    }

    return leftFinished && rightFinished;
  }
}
