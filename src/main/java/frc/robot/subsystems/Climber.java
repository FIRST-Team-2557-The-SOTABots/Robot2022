// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private boolean stallProtectionOn;
  private Timer stallTimer;

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

    stallProtectionOn = false;
    stallTimer = new Timer();
    stallTimer.reset();

    lock();
  }



  /**
   * Locks the angle motor. The motor cannot run when locked.
   */
  public void lock(){
    angleLock.set(LOCK_VALUE);
  }



  /**
   * Unlocks the angle motor
   */
  public void unlock(){
    angleLock.set(UNLOCK_VALUE);
  }



  /**
   * 
   * @return whether the angle motor is locked
   */
  public boolean getLocked() {
    return angleLock.get() == LOCK_VALUE;
  }



  /**
   * Extends the left extending hook. The hook will not extend up if the top mag limit is on, 
   * or the encoder value is past some limit (in the event that the mag limit does not trigger).
   * The hook will not retract if the bottom mag limit is on.
   * @param spd the percent speed at which the hook should move. positive will extend, negative will retract.
   */
  public void extendLeftHook(double spd) {
    if (getLeftTopMagLimit())
      spd = Math.min(0, spd);

    if (getLeftBotMagLimit())
      spd = Math.max(0, spd);
    
    if (getLeftEncoderPosition() >= LIMIT_EXTEND_ENCODER_LEFT)
      spd = Math.min(0, spd);

    leftHook.set(spd);
  }



  /**
   * Extends the right extending hook. The hook will not extend up if the top mag limit is on, 
   * or the encoder value is past some limit (in the event that the mag limit does not trigger).
   * The hook will not retract if the bottom mag limit is on.
   * @param spd the percent speed at which the hook should move. positive will extend, negative will retract.
   */
  public void extendRightHook(double spd) {
    if (getRightTopMagLimit())
      spd = Math.min(0, spd);

    if (getRightBotMagLimit())
      spd = Math.max(0, spd);

    if (getRightEncoderPosition() >= LIMIT_EXTEND_ENCODER_RIGHT)
      spd = Math.min(0, spd);
    
    rightHook.set(spd);
  }



  /**
   * Retract both hooks at a slow preset speed.
   */
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



  /**
   * Runs the angle hook. The hook cannot angle past either of the maximum or minimum limits.
   * It also cannot angle if it is locked.
   * @param spd the percent speed to angle at. positive angles towards the shooter, 
   * negative angles towards the intake.
   */
  public void runAngle(double spd) {
    if (getAngleEncoderPosition() >= ANGLE_ENCODER_HIGH_LIMIT)
      spd = Math.min(0, spd);

    if (getAngleEncoderPosition() <= ANGLE_ENCODER_LOW_LIMIT)
      spd = Math.max(0, spd);

    // don't allow angle motor to move if it is locked or in stall protection
    if (getLocked() || stallProtectionOn)
      angleMotor.set(0.0);
    else
      angleMotor.set(spd);
  }



  /**
   * 
   * @return the encoder value of the left hook, with positive values meaning more extended.
   */
  public double getLeftEncoderPosition() {
    return leftEncoder.get();
  }



  /**
   * 
   * @return the encoder value of the right hook, with positive values meaning more extended.
   */
  public double getRightEncoderPosition() {
    return -rightEncoder.get();
  }



  /**
   * 
   * @return whether the left bottom mag limit is activated and the left hook is completely retracted
   */
  public boolean getLeftBotMagLimit() {
    return !leftBotMagSensor.get();
  }



  /**
   * 
   * @return whether the right bottom mag limit is activated and the right hook is completely retracted
   */
  public boolean getRightBotMagLimit() {
    return !rightBotMagSensor.get();
  }



  /**
   * 
   * @return whether the left top mag limit is activated and the left hook is completely extended
   */
  public boolean getLeftTopMagLimit() {
    return !leftTopMagSensor.get();
  }



  /**
   * 
   * @return whether the right top mag limit is activated and the right hook is completely extended
   */
  public boolean getRightTopMagLimit() {
    return !rightTopMagSensor.get();
  }



  /**
   * Gets the angle encoder's position, where the positive direction is towards the shooter and 0 is at the intake
   * side hard limit.
   * @return the angle encoder value
   */
  public double getAngleEncoderPosition() {
    return ANGLE_HOOK_INVERTED ? -angleMotor.getSelectedSensorPosition() : angleMotor.getSelectedSensorPosition();
  }



  /**
   * Resets the climb system's encoder values to 0.
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
    angleMotor.setSelectedSensorPosition(MIN_ANGLE_ENCODER);
  }



  /**
   * Sets the angle motor to coast
   */
  public void setAngleMotorCoast() {
    angleMotor.setNeutralMode(NeutralMode.Coast);
  }



  /**
   * Sets the angle motor to brake
   */
  public void setAngleMotorBrake() {
    angleMotor.setNeutralMode(NeutralMode.Brake);
  }



  /**
   * Update whether the angle motor is in stall protection. This checks whether the current being drawn is too
   * close to stall and prevents motor smoke.
   */
  private void updateStallProtection() {
    // if stall protection has been on for long enough, turn it off and reset and stop the timer
    if (stallProtectionOn && stallTimer.get() >= STALL_PROTECTION_DURATION) {
      stallProtectionOn = false;
      stallTimer.reset();
      stallTimer.stop();
    } 
    // if stall protection is 
    else if (Math.abs(angleMotor.getSupplyCurrent()) > STALL_PROTECTION_CURRENT) {
      stallProtectionOn = true;
      stallTimer.start();
      angleMotor.set(0.0);
    }
  }



  /**
   * Checks whether stall protection is active.
   * @return whether stall protection is on.
   */
  public boolean getStallProtectionOn() {
    return stallProtectionOn;
  }


  
  @Override
  public void periodic() {
    // if either the left or right bottom mag limit are tripped, reset the corresponding hook's encoder value
    if (getLeftBotMagLimit())
      leftEncoder.reset();
    
    if (getRightBotMagLimit())
      rightEncoder.reset();

    // update whether stall protection is active
    updateStallProtection();

    // log sensor states
    SmartDashboard.putNumber("left encoder", getLeftEncoderPosition());
    SmartDashboard.putNumber("right encoder", getRightEncoderPosition());
    SmartDashboard.putNumber("angle encoder", getAngleEncoderPosition());
    SmartDashboard.putBoolean("left bot", getLeftBotMagLimit());
    SmartDashboard.putBoolean("left top", getLeftTopMagLimit());
    SmartDashboard.putBoolean("right bot", getRightBotMagLimit());
    SmartDashboard.putBoolean("right top", getRightTopMagLimit());
    SmartDashboard.putBoolean("is locked", getLocked());

    // log stall-related information about the angle motor 
    SmartDashboard.putNumber("Angle power", angleMotor.get());
    // System.out.println("Stator Current" + angleMotor.getStatorCurrent());
    System.out.println("Input Current " + angleMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Angle Temperature", angleMotor.getTemperature());
    SmartDashboard.putBoolean("Stall Protection", stallProtectionOn);
  }
}
