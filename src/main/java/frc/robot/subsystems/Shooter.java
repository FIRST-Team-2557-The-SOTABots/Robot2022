// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Shooter.*;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  private DoubleSolenoid hoodSolenoid;
  private CANSparkMax motor1;
  private CANSparkMax motor2;

  private SimpleMotorFeedforward feedforward;
  private PIDController speedPID;

  private ArrayList<Double> speedSample;

  /** Creates a new Shooter. */
  public Shooter() {
    hoodSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARD_CHANNEL_PORT, REVERSE_CHANNEL_PORT);
    
    motor1 = new CANSparkMax(MOTOR_1_PORT, MotorType.kBrushless);
    motor1.restoreFactoryDefaults();
    motor1.setIdleMode(IdleMode.kCoast);
    motor1.setInverted(MOTOR_1_INVERTED);
    motor1.setOpenLoopRampRate(RAMP_RATE);
    motor2 = new CANSparkMax(MOTOR_2_PORT, MotorType.kBrushless);
    motor2.restoreFactoryDefaults();
    motor2.setIdleMode(IdleMode.kCoast);
    motor2.setInverted(MOTOR_2_INVERTED);
    motor2.setOpenLoopRampRate(RAMP_RATE);

    feedforward = new SimpleMotorFeedforward(FEEDFORWARD_KS, FEEDFORWARD_KV);
    speedPID = new PIDController(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD);

    speedSample = new ArrayList<>();
    speedSample.add(0.0);

    hoodDown();
  }

  /**
   * Sets the speed of the motor as opposed to {@link #setMotorRPM(double)}
   * @param spd Speed of the motor from -1 to 1
   */
  public void runFlywheel(double spd) {
    motor1.set(spd);
    motor2.set(spd);
  }

  /**
   * Sets the RPM of the motor with PID and FF
   * @param rpm Setpoint for the motor's RPM
   */
  public void setMotorRPM(double rpm) {
    double motorInput = feedforward.calculate(rpm) + speedPID.calculate(getMotorRPM(), rpm);
    motor1.setVoltage(motorInput);
    motor2.setVoltage(motorInput);
    speedPID.setSetpoint(rpm);
  }

  /**
   * Checks if it is ready to shoot with the average speed of the flywheel
   * @return Whether or not it is ready to shoot
   */
  public boolean readyToShoot() {
    double averageSpeed = 0.0;
    for (int i = 0; i < speedSample.size(); i++) {
      averageSpeed += speedSample.get(i);
    }
    averageSpeed /= speedSample.size();

    return Math.abs(averageSpeed - speedPID.getSetpoint()) < RPM_TOLERANCE;
  }

  /**
   * Updates the speed sample for the flywheel speed to be used in {@link #readyToShoot()}
   */
  private void updateSpeedSample() {
    speedSample.add(getMotorRPM());
    if (speedSample.size() > SPEED_SAMPLE_SIZE_LIMIT) {
      speedSample.remove(0);
    }
  }

  /**
   * Calculates the RPM of the flywheel from distance from the limelight
   * @param distance Distance in limelight ty
   * @return the RPM of the flywheel from distance
   */
  public double calculateRPM(double distance) {
    return RPM_EQUATION.apply(distance);

  }
 
  public double getMotorRPM(){
    return Math.abs(motor1.getEncoder().getVelocity());
  }

  public void hoodUp() {
    hoodSolenoid.set(RAISED_VALUE);
  }

  public void hoodDown() {
    hoodSolenoid.set(LOWERED_VALUE);
  }

  @Override
  public void periodic() {
    updateSpeedSample();
  }
}