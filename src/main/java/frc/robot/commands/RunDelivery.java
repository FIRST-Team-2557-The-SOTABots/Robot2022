// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Delivery;

public class RunDelivery extends CommandBase {
  private boolean photoTurnedOn;
  private Delivery delivery;

  public RunDelivery(Delivery delivery) {
    addRequirements(delivery);
    photoTurnedOn = false;
    this.delivery = delivery;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    photoTurnedOn = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    delivery.runMotor(Constants.Delivery.INDEXING_SPEED);
    
    if(delivery.getSensor2()){
      photoTurnedOn = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    delivery.runMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(photoTurnedOn && !delivery.getSensor2()){
    //   return true;
    // }
    return false;
  }
}