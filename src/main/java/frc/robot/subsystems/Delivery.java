// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Delivery.*;

public class Delivery extends SubsystemBase {
  /** Creates a new Sensors. */
  
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 sensor1 = new ColorSensorV3(i2cPort);
  private DigitalInput sensor2 = new DigitalInput(SENSOR_2_PORT);

  private WPI_TalonSRX deliveryMotor;

  public Delivery() {
    deliveryMotor = new WPI_TalonSRX(MOTOR_PORT);
    deliveryMotor.configFactoryDefault();
    deliveryMotor.setInverted(MOTOR_INVERTED);
  }
  
  public void runMotor(double speed) {
    deliveryMotor.set(speed);
  }

  public boolean getSensor1() {
    return sensor1.getIR() > SENSOR_1_THRESHOLD;
  }
  
  public boolean getSensor2() {
    return sensor2.get();
  }

  @Override
  public void periodic() {
  }
}