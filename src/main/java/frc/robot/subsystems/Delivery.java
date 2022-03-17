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

  private DigitalInput sensor1;
  private boolean sensor1Previous;
  private boolean sensor1Changed;
  private DigitalInput sensor2;

  public Delivery() {
    deliveryMotor = new WPI_TalonSRX(MOTOR_PORT);
    deliveryMotor.configFactoryDefault();
    deliveryMotor.setInverted(MOTOR_INVERTED);
    deliveryMotor.setNeutralMode(NeutralMode.Brake);

    sensor1 = new DigitalInput(SENSOR_1_PORT);
    sensor1Previous = false;
    sensor1Changed = false;
    sensor2 = new DigitalInput(SENSOR_2_PORT);
  }
  
  public void runMotor(double speed) {
    deliveryMotor.set(speed);
  }

  /**
   * Returns true when the state of sensor 1 has changed
   * @return whether the sensor state changed this loop
   */
  public boolean getSensor1() {
    return sensor1Changed;
  }
  
  public boolean getSensor2() {
    return sensor2.get();
  }

  @Override
  public void periodic() {
    if (sensor1Previous != sensor1.get()) {
      sensor1Changed = true;
    } else {
      sensor1Changed = false;
    }
    sensor1Previous = sensor1.get();
  }
}