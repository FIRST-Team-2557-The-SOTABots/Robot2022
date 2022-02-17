// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class RunDelivery extends CommandBase {
  private boolean motorIsRunning;
  private boolean photoTurnedOn;
  /** Creates a new RunDelivery. */
  // private Sensors sensors = new Sensors();

  public RunDelivery() {
    // Use addRequirements() here to declare subsystem dependencies.
    // this.sensors = sensors;
    addRequirements(RobotContainer.sensors);
    motorIsRunning = false;
    photoTurnedOn = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motorIsRunning = false;
    photoTurnedOn = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotContainer.sensors.getBallColor().equals("no ball") || motorIsRunning) {
      motorIsRunning = true;
      RobotContainer.sensors.runMotor(-0.4);
      if(RobotContainer.sensors.photoDetected()){
        photoTurnedOn = true;
      }
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorIsRunning = false;
    RobotContainer.sensors.runMotor(0.0);
    photoTurnedOn = false;
    SmartDashboard.putBoolean("Motor is running", motorIsRunning);
    SmartDashboard.putBoolean("Photo turned on", photoTurnedOn);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(photoTurnedOn && !RobotContainer.sensors.photoDetected()){
      return true;
    }
    return false;
  }
}