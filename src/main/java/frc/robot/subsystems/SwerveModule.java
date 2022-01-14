// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MPIDController;
import frc.robot.util.MPIDSubsystem;
import static frc.robot.Constants.Swerve.*;

public class SwerveModule extends MPIDSubsystem {
  
  private TalonFX speedMotor;
  private CANSparkMax angleMotor;
  private AnalogInput angleEncoder;
  private SwerveDrive swerveDrive;

  private int moduleNumber;
  private double speed;

  /** Creates a new SwerveModule. */
  public SwerveModule(int moduleNumber, SwerveDrive swerveDrive) {
    // initialize a pid controller using the super constructor
    super(new MPIDController(AnglePID.KP, AnglePID.KI, AnglePID.KD));

    this.moduleNumber = moduleNumber;

    // NOTE: PIDController deals in native units
    // NOTE: positive rotation is clockwise

    // make continuous input enabled, since the motor can rotate past the extreme encoder count values
    getController().enableContinuousInput(0, AngleEncoder.CPR);
    getController().setTolerance(AnglePID.TOLERANCE);

    speedMotor = new TalonFX(Ports.SPEED_MOTORS[moduleNumber]);
    angleMotor = new CANSparkMax(Ports.ANGLE_MOTORS[moduleNumber], MotorType.kBrushless);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleEncoder = new AnalogInput(Ports.ANGLE_ENCODERS[moduleNumber]);

    this.swerveDrive = swerveDrive;

    speed = 0.0;
  }



   /**
   * This method is called by the managing swerve drivetrain
   * @param setpoint module rotation setpoint in native units
   * @param speed target speed in m/s
   */
  public void drive(double setpoint, double speed) {
    setSetpoint(setpoint);
    setSpeed(speed);

    if (setpointAdjustmentNecessary()) {
      flipSetpoint();
      setSpeed(-speed);
    }
  }



  /**
   * Sets the speed of the wheel
   * @param metersPerSecond how fast the wheel should rotate in meters per second
   */
  private void setSpeed(double metersPerSecond) {
    // TODO: Uncomment once speed PIDF configured
    // // we use the velocity mode for setting motor speed, which requires motor counts per 100 ms as the speed parameter
    // double motorCountsPerDecisecond = metersPerSecond / getMetersPerCount() / 10.0;
    // System.out.println(motorCountsPerDecisecond);
    // // speedMotor.set(ControlMode.Velocity, motorCountsPerDecisecond);

    speedMotor.set(ControlMode.PercentOutput, metersPerSecond / MAX_WHEEL_SPEED);

    speed = metersPerSecond;
  }



  /**
   * The maximum angle the swerve module should have to travel to get to the right orientation
   * is 90 degrees, π / 2 radians, etc. If traveling to the setpoint would require exceeding this 
   * amount, it would be faster to rotate to the angle opposite the setpoint. This method checks
   * whether this adjustment is needed.
   * @return whether this setpoint needs adjustment
   */
  public boolean setpointAdjustmentNecessary() {
    double currentSetpoint = getSetpoint();
    double currentPosition = angleEncoder.getAverageVoltage();

    // subtract the current position from the current setpoint to obtain the angle difference 
    // the easiest way to determine whether the angle difference exceeds 1/4 a rotation is to test 
    // whether the cosine of the angle is negative after converting the angle difference to radians
    // this method makes it easier to handle the continuous nature of angles
    double absAngleDifference = nativeToRadians(Math.abs(currentSetpoint - currentPosition));

    if (Math.cos(absAngleDifference) < 0) {
      return true;
    }
    
    // if the angle difference was under a quarter turn, the setpoint doesn't need to change 
    return false;
  }



  /**
   * If the setpoint needs to be adjusted so the module takes a faster route to an angle,
   * this method can readjust the setpoint to be the opposite angle
   */
  private void flipSetpoint() {
    double currentSetpoint = getSetpoint();
    double halfEncoderCircle = AngleEncoder.CPR / 2;

    // we need to flip the setpoint by half a circle, but we need to keep it within the range of
    // valid angle encoder values, so we make checks to determine whether to add or subtract a half
    // circle
    if (currentSetpoint < halfEncoderCircle) {
      setSetpoint(currentSetpoint + halfEncoderCircle);
    } else {
      setSetpoint(currentSetpoint - halfEncoderCircle);
    }
  }



  /**
   * @param encoderCounts number of encoder counts
   * @return absolute encoder counts converted to radians
   */
  public static double nativeToRadians(double encoderCounts) {
    return encoderCounts * 2 * Math.PI / AngleEncoder.CPR;
  }



  /**
   * 
   * @param radians number of radians
   * @return radians converted to absolute encoder counts
   */
  public static double radiansToNative(double radians) {
    return radians / (2 * Math.PI) * AngleEncoder.CPR;
  }  
  
  
  
  /**
  * 
  * @return meters per count of talon fx encoder based on current gear
  */
  public double getMetersPerCount() {
    //     Circumference    /        Gear Ratio       / Counts Per Motor Revolution = 
    // meters per rev wheel / rev motor per rev wheel /    counts per rev motor     = Meters Per Encoder Count
    return WHEEL_CIRCUMFERENCE / GearRatios.DRIVE[swerveDrive.getCurrentGear()] / TalonEncoder.CPR;
  }



  /**
   * 
   * @return the wheel speed of the module is set to in meters per second
   */
  public double getSetSpeed() {
    return speed;
  }

  

  /**
   * 
   * @return the calculated wheel speed of the module in meters per second
   */
  public double getSpeed() {
    //   Motor Velocity  * Time Conversion * Distance Per Count = 
    // Counts Per 100 ms * 1000 ms Per 1 s *  Meters Per Count  = Wheel Speed
    return speedMotor.getSensorCollection().getIntegratedSensorVelocity();
  }



  /**
   * 
   * @return the current gear of the module
   */
  public int getCurrentGear() {
      return swerveDrive.getCurrentGear();
  }



  @Override
  public void useOutput(double output, double setpoint) {
    angleMotor.set(output);
  }



  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return angleEncoder.getAverageVoltage();
  }



  @Override
  public void periodic() {
    super.periodic();
    // SmartDashboard.putNumber("Module " + this.moduleNumber + " setpoint", this.getSetpoint());
    // SmartDashboard.putBoolean("Module " + this.moduleNumber + " at setpoint", this.getController().atSetpoint());

    SmartDashboard.putNumber("Module " + this.moduleNumber + " angle", this.angleEncoder.getAverageVoltage());
    SmartDashboard.putNumber("Module " + this.moduleNumber + " gear", this.getCurrentGear());
    SmartDashboard.putNumber("Module " + this.moduleNumber + " speed", this.getSpeed());
  }
}
