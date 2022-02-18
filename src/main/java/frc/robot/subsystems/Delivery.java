// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Delivery extends SubsystemBase {
  /** Creates a new Sensors. */
  // private final AnalogInput ultrasonic = new AnalogInput(5);
  // private double rawValue = ultrasonic.getValue();
  // private double voltage_scale_factor;
  // private double currentDistanceCentimeters;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final DigitalInput photoElectricSensor = new DigitalInput(0);

  private boolean colorDetected;
  private boolean photoElectricDetected;

  private WPI_TalonSRX deliveryMotor;

  public Delivery() {
    //matcher.addColorMatch(cargoBlue);
    //matcher.addColorMatch(cargoRed);
    //matcher.addColorMatch(air);
    deliveryMotor = new WPI_TalonSRX(4);
    deliveryMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // rawValue = ultrasonic.getValue();
    // voltage_scale_factor = 5/RobotController.getVoltage5V();
    // currentDistanceCentimeters = rawValue * voltage_scale_factor * 0.125;
    // SmartDashboard.putNumber("Raw Value", rawValue);
    // SmartDashboard.putNumber("Distance", currentDistanceCentimeters);
    // SmartDashboard.putNumber("voltage scale", voltage_scale_factor);
    SmartDashboard.putNumber("Red", colorSensor.getRed());
    SmartDashboard.putNumber("Green", colorSensor.getGreen());
    SmartDashboard.putNumber("Blue", colorSensor.getBlue());
    SmartDashboard.putString("Cargo Color", getBallColor());
    SmartDashboard.putNumber("proximity", colorSensor.getProximity());
    SmartDashboard.putBoolean("new sensor", photoElectricSensor.get());

    updateColorDetection();
    updatePhotoElectricDetection();

  }
  
  public String getBallColor(){
    if(colorSensor.getRed() >= 9000){
      return "red";
    } else if(colorSensor.getBlue() >= 9000){
      return "blue";
    } else {
      return "no ball";
    }
  }

  public boolean colorDetected(){
    return colorDetected;
  }

  public boolean photoDetected(){
    return photoElectricDetected;
  }

  private void updateColorDetection(){
    if(!getBallColor().equals("no ball")){
      colorDetected = true;
    }
    colorDetected = false;
  }

  private void updatePhotoElectricDetection(){
    photoElectricDetected = photoElectricSensor.get();
  }

  public void runMotor(double speed) {
    deliveryMotor.set(speed);
  }

  // public String getDetectedColor(){
  //   detectedColor = colorSensor.getColor();
  //   matchResult = matcher.matchClosestColor(detectedColor);
  //   if(matchResult.color == cargoRed){
  //     return "Red";
  //   } else if(matchResult.color == cargoBlue){
  //     return "Blue";
  //   } else if(matchResult.color == air) {
  //     return "Air";
  //   } else {
  //     return "Unknown";
  //   }

}