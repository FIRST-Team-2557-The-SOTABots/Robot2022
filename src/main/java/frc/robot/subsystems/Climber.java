// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climber.*;

public class Climber extends SubsystemBase {
  private CANSparkMax leftHook;
  private CANSparkMax rightHook;
  private WPI_TalonSRX angleMotor;
  private DoubleSolenoid angleLock;

  private DigitalInput leftMagSensor;
  private DigitalInput rightMagSensor; //TODO: dont actually know if there is going to be a second mag sensor

  private DutyCycleEncoder leftEncoder;
  private DutyCycleEncoder rightEncoder;
  private DutyCycleEncoder angleEncoder;

  /** Creates a new Climber. */
  public Climber() {
    leftHook = new CANSparkMax(LEFT_HOOK_MOTOR_PORT, MotorType.kBrushless);
    leftHook.restoreFactoryDefaults();
    leftHook.getEncoder().setPosition(MIN_EXTEND_ENCODER);
    leftHook.setInverted(LEFT_HOOK_INVERTED);
    leftHook.setIdleMode(IdleMode.kBrake);
    rightHook = new CANSparkMax(RIGHT_HOOK_MOTOR_PORT, MotorType.kBrushless);
    rightHook.restoreFactoryDefaults();
    rightHook.getEncoder().setPosition(MIN_EXTEND_ENCODER);
    rightHook.setInverted(RIGHT_HOOK_INVERTED);
    rightHook.setIdleMode(IdleMode.kBrake);
    angleMotor = new WPI_TalonSRX(ANGLE_MOTOR_PORT);
    angleMotor.configFactoryDefault();
    angleMotor.setSelectedSensorPosition(MIN_ANGLE_ENCODER);
    angleMotor.setInverted(ANGLE_HOOK_INVERTED);
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleLock = new DoubleSolenoid(PneumaticsModuleType.REVPH, SOLENOID_CHANNEL_A, SOLENOID_CHANNEL_B);
    
    leftMagSensor = new DigitalInput(RIGHT_MAG_SENSOR_PORT);
    rightMagSensor = new DigitalInput(LEFT_MAG_SENSOR_PORT);

    leftEncoder = new DutyCycleEncoder(LEFT_HOOK_ENCODER_PORT);
    leftEncoder.reset();
    rightEncoder = new DutyCycleEncoder(RIGHT_HOOK_ENCODER_PORT);
    rightEncoder.reset();

    // angleEncoder = new DutyCycleEncoder(ANGLE_ENCODER_PORT);
    // angleEncoder.reset();

    unlock(); // TODO: switch back
  }

  public void lock(){
    angleLock.set(LOCK_VALUE);
  }

  public void unlock(){
    angleLock.set(UNLOCK_VALUE);
  }

  public void extendLeftHook(double spd) {
    if (getLeftEncoderPosition() >= EXTEND_HIGH_LIMIT)
      spd = Math.min(0, spd);

    if (getLeftEncoderPosition() <= EXTEND_LOW_LIMIT || getLeftMagLimit())
      spd = Math.max(0, spd);

    leftHook.set(spd);
  }

  public void extendRightHook(double spd) {
    if (getRightEncoderPosition() >= EXTEND_HIGH_LIMIT)
      spd = Math.min(0, spd);

    if (getRightEncoderPosition() <= EXTEND_LOW_LIMIT || getRightMagLimit())
      spd = Math.max(0, spd);
    
    rightHook.set(spd);
  }

  public void runAngle(double spd) {
    if (getAngleEncoderPosition() >= ANGLE_ENCODER_HIGH_LIMIT)
      spd = Math.min(0, spd);

    if (getAngleEncoderPosition() <= ANGLE_ENCODER_LOW_LIMIT)
      spd = Math.max(0, spd);

    angleMotor.set(spd);
  }

  public double getLeftEncoderPosition() {
    return leftEncoder.get();
  }

  public double getRightEncoderPosition() {
    return rightEncoder.get();
  }

  public boolean getLeftMagLimit() {
    return !leftMagSensor.get();
  }

  public boolean getRightMagLimit() {
    return !rightMagSensor.get();
  }

  public double getAngleEncoderPosition() {
    return angleMotor.getSelectedSensorPosition();
  }



  // /**
  //  * 
  //  * @return angle of the angling arm in radians
  //  */
  // public double getAngle() {
  //   return (MAX_ANGLE - MIN_ANGLE) / (MAX_ANGLE_ENCODER - MIN_ANGLE_ENCODER) * getAngleEncoderPosition() + MIN_ANGLE;
  // }

  /**
   * 
   * @return length of extending arm from the pivot point to the hook in meters
   */
  // public double getLength() {

  //   // TODO make return length of the extending arms 
  //   return (MAX_EXTEND_HOOK_LENGTH - MIN_EXTEND_HOOK_LENGTH) / (MAX_EXTEND_HOOK_ENCODER - MIN_EXTEND_HOOK_ENCODER) * getExtendEncoderPosition() + MIN_EXTEND_HOOK_LENGTH;

  // }

  // /**
  //  * 
  //  * @param angle in radians
  //  * @return returns length of the extending arms from the angle of the angling arms
  //  */
  // public double getReqLength(double a) {
  //   return ANGLE_HOOK_LENGTH * Math.cos(a) + 
  //     Math.sqrt(
  //       Math.pow(DISTANCE_BETWEEN_BARS, 2) - 
  //       Math.pow(ANGLE_HOOK_LENGTH, 2) * Math.pow(Math.sin(a), 2)
  //     );
  // }

  public void reset() {
    // lock();
    // leftHook.getEncoder().setPosition(MIN_EXTEND_HOOK_ENCODER);
    // rightHook.getEncoder().setPosition(MIN_EXTEND_HOOK_ENCODER);
    leftEncoder.reset();
    rightEncoder.reset();
    angleMotor.setSelectedSensorPosition(MIN_ANGLE_ENCODER);
  }

  public PIDCommand generateAnglePIDCommand(double setpoint) {
    PIDCommand result = new PIDCommand(
      new PIDController(ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD),
      this::getAngleEncoderPosition, 
      setpoint,
      this::runAngle, 
      this
    );
    result.getController().setTolerance(ANGLE_PID_TOLERANCE);
    return result;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("left mag", leftMagSensor.get());
    SmartDashboard.putBoolean("right mag", rightMagSensor.get());
    SmartDashboard.putNumber("left encoder", getLeftEncoderPosition());
    SmartDashboard.putNumber("right encoder", getRightEncoderPosition());
    SmartDashboard.putNumber("angle encoder", getAngleEncoderPosition());
  }
}
