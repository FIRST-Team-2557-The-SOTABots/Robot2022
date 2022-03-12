// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LimeLight.*;

public class Limelight extends SubsystemBase {

  static NetworkTable table;

  private double x;
  private double y;
  private double valid;

  /** Creates a new Limelight. */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /** Updates the camera data from the limelight stream */
  public void getCamData() {
    x = table.getEntry("tx").getDouble(0.0); // x offset angle of the target's center
    y = table.getEntry("ty").getDouble(0.0); // y offset angle of the target's center
    valid = table.getEntry("tv").getDouble(0.0); // 0 if no target detected, 1 if detected
  }

  /**
   * Returns true if a vision target is detected
   * @return whether target is detected
   */
  public boolean targetDetected(){
    return (valid == 1) ? true : false;
  }

  /**
   * Gets the x error to the vision target. Use this for rotating towards the target
   * @return x error in degrees
   */
  public double getX(){
    return x;
  }

  /**
   * Gets y angle to the vision target
   * @return y angle in degrees
   */
  public double getY(){
    return y;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getCamData();

    // String shootStatus = "";
    if (!targetDetected()) {
      SmartDashboard.putString("Shoot Status", "No target");
    } else if (getY() < MIN_TY) {
      SmartDashboard.putString("Shoot Status", "Too far");
    } else {
      SmartDashboard.putString("Shoot Status", new String(Character.toChars(0x1F604)) + new String(Character.toChars(0x1F44D)));
    }

    // SmartDashboard.putNumber("tx", getX());

  }
}