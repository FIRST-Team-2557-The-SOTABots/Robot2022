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
  private MovementType movementType;

  /** Creates a new ExtendClimbToPosition. */
  public ExtendClimbToPosition(MovementType movementType, Climber climber) {
    this.movementType = movementType;

    double leftGoal = movementType.leftGoal;
    double rightGoal = movementType.rightGoal;
    
    leftController = new ProfiledPIDController(
      EXTEND_PID_KP, EXTEND_PID_KI, EXTEND_PID_KD, 
      new TrapezoidProfile.Constraints(EXTEND_MAX_VELOCITY_LEFT, EXTEND_MAX_ACCELERATION_LEFT)
    );
    leftController.setGoal(leftGoal);
    leftController.setTolerance(EXTEND_PID_TOLERANCE);
    rightController = new ProfiledPIDController(
      EXTEND_PID_KP, EXTEND_PID_KI, EXTEND_PID_KD, 
      new TrapezoidProfile.Constraints(EXTEND_MAX_VELOCITY_RIGHT, EXTEND_MAX_ACCELERATION_RIGHT)
    );
    rightController.setGoal(rightGoal);
    rightController.setTolerance(EXTEND_PID_TOLERANCE);
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
    SmartDashboard.putNumber("left position", climber.getLeftEncoderPosition());
    
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
    boolean leftFinished;
    boolean rightFinished;
    if (movementType == MovementType.MID_TO_BOTTOM || movementType == MovementType.TOP_TO_BOTTOM) {
      leftFinished = climber.getLeftBotMagLimit();
      rightFinished = climber.getRightBotMagLimit();
    } else if (movementType == MovementType.BOTTOM_TO_TOP || movementType == MovementType.MID_TO_TOP) {
      leftFinished = climber.getLeftTopMagLimit();
      rightFinished = climber.getRightTopMagLimit();
    } else {
      leftFinished = leftController.atGoal();
      rightFinished = rightController.atGoal();
    }

    return leftFinished && rightFinished;
  }
}
