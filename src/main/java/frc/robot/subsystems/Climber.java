// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climber.*;

public class Climber extends SubsystemBase {
  private CANSparkMax leftHook;
  private CANSparkMax rightHook;
  private WPI_TalonSRX angleMotor;
  private DoubleSolenoid angleLock;

  /** Creates a new Climber. */
  public Climber() {
    leftHook = new CANSparkMax(LEFT_HOOK_MOTOR_PORT, MotorType.kBrushless);
    leftHook.restoreFactoryDefaults();
    leftHook.getEncoder().setPosition(HOOK_START_POSITION);
    leftHook.setInverted(LEFT_HOOK_INVERTED);
    rightHook = new CANSparkMax(RIGHT_HOOK_MOTOR_PORT, MotorType.kBrushless);
    rightHook.restoreFactoryDefaults();
    rightHook.getEncoder().setPosition(HOOK_START_POSITION);
    rightHook.follow(leftHook, RIGHT_HOOK_INVERTED);
    angleMotor = new WPI_TalonSRX(ANGLE_MOTOR_PORT);
    angleMotor.configFactoryDefault();
    angleMotor.getSensorCollection().setQuadraturePosition(ANGLE_HOOK_START_POSITION, 0);
    angleMotor.setInverted(ANGLE_HOOK_INVERTED);
    angleLock = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARD_CHANNEL, REVERSE_CHANNEL);
    lock();
  }

  public void lock(){
    angleLock.set(LOCK_VALUE);
  }

  public void unlock(){
    angleLock.set(UNLOCK_VALUE);
  }

  public void runExtend(double spd) {

    if (getExtendEncoderPosition() >= EXTEND_HIGH_LIMIT)
      spd = Math.min(0, spd);

    if (getExtendEncoderPosition() <= EXTEND_LOW_LIMIT)
      spd = Math.max(0, spd);

    leftHook.set(spd);

  }

  public void runAngle(double spd) {

    if (getAngleEncoderPosition() >= ANGLE_HIGH_LIMIT)
      spd = Math.min(0, spd);

    if (getAngleEncoderPosition() <= ANGLE_LOW_LIMIT)
      spd = Math.max(0, spd);

    angleMotor.set(spd);

  }

  public double getExtendEncoderPosition() {

    return leftHook.getEncoder().getPosition();

  }

  public int getAngleEncoderPosition() {

    return angleMotor.getSensorCollection().getQuadraturePosition();

  }

  /**
   * 
   * @return angle of the angling arm in radians
   */
  public double getAngle() {

    // TODO make return angle based off of angleMotors encoder count
    return 0.0;

  }

  /**
   * 
   * @return length of extending arm from the pivot point to the hook
   */
  public double getLength() {

    // TODO make return length of the extending arms 
    return 0.0;

  }

  /**
   * 
   * @param angle in radians
   * @return returns length of the extending arms from the angle of the angling arms
   */
  public double getReqLength(double a) {

    // TODO make return required length from angle(in radians)
    return ANGLE_HOOK_LENGTH * Math.cos(a) + Math.sqrt(Math.pow(DISTANCE_BETWEEN_BARS, 2)); // TODO NOT DONE!!!!

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
