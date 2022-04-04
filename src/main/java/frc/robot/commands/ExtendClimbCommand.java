// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.Climber.*;
import frc.robot.subsystems.Climber;

public class ExtendClimbCommand extends CommandBase {

  private Climber climber;
  private SimpleExtendMovement movement;

  /**
   * 
   * @param climber the climber subsystem to use
   * @param movement the movement to execute
   */
  public ExtendClimbCommand(Climber climber, SimpleExtendMovement movement) {
    addRequirements(climber);
    this.climber = climber;
    this.movement = movement;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Extend Command", "Initializing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Extend Command", "Executing");
    //sets the speeds based on movement type
    if (movement == SimpleExtendMovement.BOTTOM_TO_TOP || movement == SimpleExtendMovement.MID_TO_TOP) {
      climber.extendLeftHook(movement.speed);
      climber.extendRightHook(movement.speed);
    } else if (
      movement == SimpleExtendMovement.TOP_TO_BOTTOM || 
      movement == SimpleExtendMovement.HIGH_TO_BOTTOM ||
      movement == SimpleExtendMovement.MID_TO_BOTTOM
    ) {
      climber.extendLeftHook(-movement.speed);
      climber.extendRightHook(-movement.speed);
    } else {
      if (movement.leftSetpoint - climber.getLeftEncoderPosition() > 0) {
        climber.extendLeftHook(movement.speed);
      } else {
        climber.extendLeftHook(-movement.speed);
      }
      
      if (movement.rightSetpoint - climber.getRightEncoderPosition() > 0) {
        climber.extendRightHook(movement.speed);
      } else {
        climber.extendRightHook(-movement.speed);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Extend Command", "End");
    climber.extendLeftHook(0.0);
    climber.extendRightHook(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (movement) {
      case BOTTOM_TO_TOP:
      case MID_TO_TOP:
        return climber.getLeftTopMagLimit() && climber.getRightTopMagLimit();
      case TOP_TO_BOTTOM:
      case HIGH_TO_BOTTOM:
      case MID_TO_BOTTOM:
        return climber.getLeftBotMagLimit() && climber.getRightBotMagLimit();
      default:
        return 
          Math.abs(movement.leftSetpoint - climber.getLeftEncoderPosition()) < movement.tolerance &&
          Math.abs(movement.rightSetpoint - climber.getRightEncoderPosition()) < movement.tolerance;
    }
    
  }
}
