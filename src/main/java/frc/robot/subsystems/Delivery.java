// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Delivery.*;

public class Delivery extends SubsystemBase {
  /** Creates a new Sensors. */
  
  private WPI_TalonSRX deliveryMotor;

  // Top distance sensor
  private DigitalInput sensor1;
  // Bottom IR (??) sensor
  private DigitalInput sensor2;

  public Delivery() {
    deliveryMotor = new WPI_TalonSRX(MOTOR_PORT);
    deliveryMotor.configFactoryDefault();
    deliveryMotor.setInverted(MOTOR_INVERTED);
    deliveryMotor.setNeutralMode(NeutralMode.Brake);
    deliveryMotor.configOpenloopRamp(RAMP_RATE);

    sensor1 = new DigitalInput(SENSOR_1_PORT);
    sensor2 = new DigitalInput(SENSOR_2_PORT);
  }
  
  public void runMotor(double speed) {
    deliveryMotor.set(speed);
  }

  /**
   * Returns true when the state of sensor 1 has changed
   * @return Whether the sensor state changed this loop
   */
  public boolean getSensor1() {
    return !sensor1.get();
  }
  
  /**
   * Returns the state of the bottom sensor
   * @return The state of the bottom sensor
   */
  public boolean getSensor2() {
    return sensor2.get();
  }

  @Override
  public void periodic() {
  }
}