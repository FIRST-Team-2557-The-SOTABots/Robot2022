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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {

  private CANSparkMax motor;
  private DoubleSolenoid intakeSolenoid;
  private Value previousValue;
  private Timer stateUpdateTimer;
  private boolean isRetracted;

  /** Creates a new Intake. */
  public Intake() {
    motor = new CANSparkMax(MOTOR_PORT, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(MOTOR_INVERTED);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, SOLENOID_CHANNEL_A, SOLENOID_CHANNEL_B);
    retract();
    previousValue = RETRACT_VALUE;
    stateUpdateTimer = new Timer();
    isRetracted = true;
  }

  /**
   * Runs the motor of the intake
   * @param speed speed of the intake motor from -1 to 1
   */
  public void run(double speed) {
    motor.set(speed);
  }

  /**
   * Extends intake
   */
  public void extend() {
    intakeSolenoid.set(EXTEND_VALUE);
  }

  /**
   * Retracts intake
   */
  public void retract() {
    intakeSolenoid.set(RETRACT_VALUE);
  }

  /**
   * Either,
   * The solenoid returns the retract value
   * Or returns off (secondary retract value ??),
   * It returns true,
   * if not then it returns the boolean {@link #isRetracted}
   * @return whether or not the intake is retracted
   */
  public boolean isRetracted() {
    return intakeSolenoid.get() == RETRACT_VALUE || intakeSolenoid.get() == Value.kOff ? true : isRetracted;
  }

  /**
   * Updates the {@link #isRetracted} on whether it is updated or not
   */
  private void updateRetracted() {
    // Starts update timer if the prev value is not the same as the current value
    if (previousValue != intakeSolenoid.get()) {
      stateUpdateTimer.reset();
      stateUpdateTimer.start();
    }

    // If the timer exceeds the extend time then update the isRetracted value
    if (stateUpdateTimer.get() > EXTEND_TIME) {
      isRetracted = intakeSolenoid.get() == RETRACT_VALUE ? true : false;
    }

    // If the intake solenoid returns a value that is not off
    // Then it sets the prev value to the current one
    // Note: the kOff value is only seen when it is disabled or set to it explicitly
    if (intakeSolenoid.get() != Value.kOff)
      previousValue = intakeSolenoid.get();
    else 
      previousValue = RETRACT_VALUE;
  }

  @Override
  public void periodic() {
    updateRetracted();
  }
}
