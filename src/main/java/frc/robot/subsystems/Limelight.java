// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LimeLight.*;

public class Limelight extends SubsystemBase {

  static NetworkTable table;
  static NetworkTableEntry tx;
  static NetworkTableEntry ty;
  static NetworkTableEntry ta;
  static NetworkTableEntry tv;

  private double x;
  private double y;
  private double area;
  private double valid;
  private double distance;

  /** Creates a new Limelight. */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getCamData();

    distance = calculateDistance(LIMELIGHT_HEIGHT, TARGET_HEIGHT, LIMELIGHT_MOUNT_ANGLE); // TODO: find real params
    SmartDashboard.putNumber("Distance to target", distance);

  }

  /** Updates the camera data from the limelight stream */
  public void getCamData() {
	  tx = table.getEntry("tx"); // x offset angle of the target's center
    ty = table.getEntry("ty"); // y offset angle of the target's center
    ta = table.getEntry("ta"); // % area the target takes of the field of vision
    tv = table.getEntry("tv"); // 0 if no target detected, 1 if detected

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    valid = tv.getDouble(0.0);

    SmartDashboard.putNumber("tx", x);
    SmartDashboard.putNumber("ty", y);
    SmartDashboard.putNumber("ta", area);
    SmartDashboard.putNumber("tv", valid);
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

  /**
   * Gets distance to a vision target
   * @return distance over ground to vision target. Returns -1 if no target detected
   */
  public double getDistance(){
    return distance;
  }

  /**
   * Calculates ground distance in centimeters to the plane of a vision target. 
   * Refer to https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   * @param camHeight height of the limelight above the ground in cm
   * @param targetHeight height of the center of the target above the ground in cm
   * @param camAngle mounting angle of the limelight on the robot in degrees
   * @return distance in cm or -1 if no target is detected
   */
  private double calculateDistance(double camHeight, double targetHeight, double camAngle){
    if(targetDetected()){
      double camAngleRads = Math.toRadians(camAngle);
      double yRads = Math.toRadians(y);
  
      double distance = (targetHeight - camHeight) / Math.tan(camAngleRads + yRads);
      return distance;
    } else {
      return -1;
    }
  }

}