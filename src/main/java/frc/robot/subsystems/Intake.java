// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {

  private CANSparkMax motor;
  private DoubleSolenoid doubleSolenoid;
  private Value previousValue;
  private Timer stateUpdateTimer;
  private boolean isRetracted;

  /** Creates a new Intake. */
  public Intake() {
    motor = new CANSparkMax(MOTOR_PORT, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(MOTOR_INVERTED);
    doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, SOLENOID_CHANNEL_A, SOLENOID_CHANNEL_B);
    retract();
    previousValue = RETRACT_VALUE;
    stateUpdateTimer = new Timer();
    isRetracted = true;
  }

  public void run(double speed) {
    motor.set(speed);
  }

  public void extend() {
    doubleSolenoid.set(EXTEND_VALUE);
  }

  public void retract() {
    doubleSolenoid.set(RETRACT_VALUE);
  }

  public boolean isRetracted() {
    return doubleSolenoid.get() == RETRACT_VALUE || doubleSolenoid.get() == Value.kOff ? true : isRetracted;
  }

  private void updateRetracted() {
    if (previousValue != doubleSolenoid.get()) {
      stateUpdateTimer.reset();
      stateUpdateTimer.start();
    }

    if (stateUpdateTimer.get() > EXTEND_TIME) {
      isRetracted = doubleSolenoid.get() == RETRACT_VALUE ? true : false;
    }

    if (doubleSolenoid.get() != Value.kOff)
      previousValue = doubleSolenoid.get();
    else 
      previousValue = RETRACT_VALUE;
  }

  @Override
  public void periodic() {
    updateRetracted();
  }
}
