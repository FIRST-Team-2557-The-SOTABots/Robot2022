// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public void runFlywheel(double spd) {
    motor1.set(spd);
    motor2.set(spd);
  }

  public void setMotorRPM(double rpm) {
    
    // speedPID.setP(SmartDashboard.getNumber("kp", SPEED_PID_KP));
    // speedPID.setI(SmartDashboard.getNumber("ki", SPEED_PID_KI));
    // speedPID.setD(SmartDashboard.getNumber("kd", SPEED_PID_KD));

    // boolean withinIzone = true;
    // reset accumulated error of PID if not within i zone so integral only active within i zone
    // if (Math.abs(getMotorRPM() - rpm) > SmartDashboard.getNumber("izone", SPEED_PID_I_ZONE)) {
    //   speedPID.reset();
    //   withinIzone = false;
    // }
    // SmartDashboard.putBoolean("withinIZone", withinIzone);

    SmartDashboard.putNumber("Setpoint", rpm);

    double motorInput = feedforward.calculate(rpm) + speedPID.calculate(getMotorRPM(), rpm);
    motor1.setVoltage(motorInput);
    motor2.setVoltage(motorInput);
    speedPID.setSetpoint(rpm);
  }

  public boolean readyToShoot() {
    double averageSpeed = 0.0;
    for (int i = 0; i < speedSample.size(); i++) {
      averageSpeed += speedSample.get(i);
    }
    averageSpeed /= speedSample.size();

    return Math.abs(averageSpeed - speedPID.getSetpoint()) < RPM_TOLERANCE;
  }

  public void setIdle() {
    this.runFlywheel(IDLE_SPEED);
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

  private void updateSpeedSample() {
    speedSample.add(getMotorRPM());
    if (speedSample.size() > SPEED_SAMPLE_SIZE_LIMIT) {
      speedSample.remove(0);
    }
  }

  public double calculateRPM(double distance) {
    // distance in ty, from limelight
    
    return RPM_EQUATION.apply(distance);
  }

  @Override
  public void periodic() {
    updateSpeedSample();
    
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("error", speedPID.getSetpoint() - getMotorRPM());
    // SmartDashboard.putNumber("speed", getMotorRPM());
  }
}