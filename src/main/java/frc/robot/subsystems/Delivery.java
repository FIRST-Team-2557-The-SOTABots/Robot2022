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

import static frc.robot.Constants.Delivery.*;

public class Delivery extends SubsystemBase {
  /** Creates a new Sensors. */
  
  private I2C multiplexer = new I2C(I2C.Port.kMXP, 0x70);
  private ColorSensorV3 sensor1Left;
  private ColorSensorV3 sensor1Right;
  private DigitalInput sensor2 = new DigitalInput(SENSOR_2_PORT);

  private WPI_TalonSRX deliveryMotor;

  public Delivery() {
    deliveryMotor = new WPI_TalonSRX(MOTOR_PORT);
    deliveryMotor.configFactoryDefault();
    deliveryMotor.setInverted(MOTOR_INVERTED);
    deliveryMotor.setNeutralMode(NeutralMode.Brake);

    multiplexer.write(0x70, 1 << SENSOR_1_LEFT_PORT); 
    sensor1Left = new ColorSensorV3(I2C.Port.kMXP);
    multiplexer.write(0x70, 1 << SENSOR_1_RIGHT_PORT); 
    sensor1Right = new ColorSensorV3(I2C.Port.kMXP);
  }
  
  public void runMotor(double speed) {
    deliveryMotor.set(speed);
  }

  public double getSensor1Left() {
    multiplexer.write(0x70, 1 << SENSOR_1_LEFT_PORT);
    return sensor1Left.getIR();

    // return 0.00;
  }

  public double getSensor1Right() {
    multiplexer.write(0x70, 1 << SENSOR_1_RIGHT_PORT);
    return sensor1Right.getIR();

    // return 0.000;
  }

  public boolean getSensor1() {
    return getSensor1Left() > SENSOR_1_LEFT_THRESHOLD || getSensor1Right() > SENSOR_1_RIGHT_THRESHOLD;

  }
  
  public boolean getSensor2() {
    return sensor2.get();
  }

  @Override
  public void periodic() {
  }
}