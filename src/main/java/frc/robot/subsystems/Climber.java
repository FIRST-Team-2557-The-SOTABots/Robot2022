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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climber.*;

public class Climber extends SubsystemBase {
  private CANSparkMax leftHook;
  private CANSparkMax rightHook;
  private WPI_TalonSRX angleMotor;
  private DoubleSolenoid angleLock;

  private DigitalInput leftBotMagSensor;
  private DigitalInput rightBotMagSensor; 
  private DigitalInput leftTopMagSensor;
  private DigitalInput rightTopMagSensor; 

  private DutyCycleEncoder leftEncoder;
  private DutyCycleEncoder rightEncoder;

  private double prevLeftPos = 0.0;
  private double prevRightPos = 0.0;

  /** Creates a new Climber. */
  public Climber() {
    leftHook = new CANSparkMax(LEFT_HOOK_MOTOR_PORT, MotorType.kBrushless);
    leftHook.restoreFactoryDefaults();
    leftHook.setInverted(LEFT_HOOK_INVERTED);
    leftHook.setIdleMode(IdleMode.kBrake);
    rightHook = new CANSparkMax(RIGHT_HOOK_MOTOR_PORT, MotorType.kBrushless);
    rightHook.restoreFactoryDefaults();
    rightHook.setInverted(RIGHT_HOOK_INVERTED);
    rightHook.setIdleMode(IdleMode.kBrake);
    angleMotor = new WPI_TalonSRX(ANGLE_MOTOR_PORT);
    angleMotor.configFactoryDefault();
    angleMotor.setSelectedSensorPosition(MIN_ANGLE_ENCODER);
    angleMotor.setInverted(ANGLE_HOOK_INVERTED);
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleLock = new DoubleSolenoid(PneumaticsModuleType.REVPH, SOLENOID_CHANNEL_A, SOLENOID_CHANNEL_B);
    
    leftBotMagSensor = new DigitalInput(LEFT_BOT_MAG_SENSOR_PORT);
    rightBotMagSensor = new DigitalInput(RIGHT_BOT_MAG_SENSOR_PORT);
    leftTopMagSensor = new DigitalInput(LEFT_TOP_MAG_SENSOR_PORT);
    rightTopMagSensor = new DigitalInput(RIGHT_TOP_MAG_SENSOR_PORT);

    leftEncoder = new DutyCycleEncoder(LEFT_HOOK_ENCODER_PORT);
    leftEncoder.reset();
    rightEncoder = new DutyCycleEncoder(RIGHT_HOOK_ENCODER_PORT);
    rightEncoder.reset();

    unlock(); // TODO: switch back
  }

  public void lock(){
    angleLock.set(LOCK_VALUE);
  }

  public void unlock(){
    angleLock.set(UNLOCK_VALUE);
  }

  public void extendLeftHook(double spd) {
    if (getLeftTopMagLimit())
      spd = Math.min(0, spd);

    if (getLeftBotMagLimit())
      spd = Math.max(0, spd);

    leftHook.set(spd);
  }

  public void extendRightHook(double spd) {
    if (getRightTopMagLimit())
      spd = Math.min(0, spd);

    if (getRightBotMagLimit())
      spd = Math.max(0, spd);
    
    rightHook.set(spd);
  }

  public void retractHooksNoEncoderLimit() {
    if (!getLeftBotMagLimit())
      leftHook.set(SLOW_RETRACT_SPEED);
    else
      leftHook.set(0);

    if (!getRightBotMagLimit())
      rightHook.set(SLOW_RETRACT_SPEED);
    else
      rightHook.set(0);
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
    return -rightEncoder.get();
  }

  public boolean getLeftBotMagLimit() {
    return !leftBotMagSensor.get();
  }

  public boolean getRightBotMagLimit() {
    return !rightBotMagSensor.get();
  }

  public boolean getLeftTopMagLimit() {
    return !leftTopMagSensor.get();
  }

  public boolean getRightTopMagLimit() {
    return !rightTopMagSensor.get();
  }

  public double getAngleEncoderPosition() {
    return -angleMotor.getSelectedSensorPosition();
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
    leftEncoder.reset();
    rightEncoder.reset();
    angleMotor.setSelectedSensorPosition(MIN_ANGLE_ENCODER);
  }

  public PIDCommand generateAnglePIDCommand(AngleMovement angleMovement) {
    PIDCommand result = new PIDCommand(
      new PIDController(angleMovement.kp, angleMovement.ki, angleMovement.kd),
      () -> this.getAngleEncoderPosition(), 
      angleMovement.setpoint,
      this::runAngle
    );
    result.getController().setTolerance(angleMovement.tolerance);
    return result;
  }

  @Override
  public void periodic() {
    if (getLeftBotMagLimit())
      leftEncoder.reset();
    
    if (getRightBotMagLimit())
      rightEncoder.reset();
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("left bot mag", getLeftBotMagLimit());
    SmartDashboard.putBoolean("right bot mag", getRightBotMagLimit());
    SmartDashboard.putBoolean("left top mag", getLeftTopMagLimit());
    SmartDashboard.putBoolean("right top mag", getRightTopMagLimit());
    SmartDashboard.putNumber("left encoder", getLeftEncoderPosition());
    SmartDashboard.putNumber("right encoder", getRightEncoderPosition());
    SmartDashboard.putNumber("angle encoder", getAngleEncoderPosition());
    SmartDashboard.putNumber("right velocity", (getRightEncoderPosition() - prevRightPos) / 0.02);
    SmartDashboard.putNumber("left velocity", (getLeftEncoderPosition() - prevLeftPos) / 0.02);
    prevLeftPos = getLeftEncoderPosition();
    prevRightPos = getRightEncoderPosition();
  }
}