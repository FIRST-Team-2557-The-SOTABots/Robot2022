// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.Swerve.*;

public class SwerveModule extends PIDSubsystem {
  
  private WPI_TalonFX speedMotor;
  private CANSparkMax angleMotor;
  private AnalogInput angleEncoder;
  private SwerveDrive swerveDrive;

  private int moduleNumber;
  private SwerveModuleState state;

  /** Creates a new SwerveModule. */
  public SwerveModule(int moduleNumber, SwerveDrive swerveDrive) {
    // initialize a pid controller using the super constructor
    super(new PIDController(ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD));

    this.moduleNumber = moduleNumber;

    // NOTE: PIDController deals in native units
    // NOTE: positive rotation is counter-clockwise

    // make continuous input enabled, since the motor can rotate past the extreme encoder count values
    getController().enableContinuousInput(0, ANGLE_ENCODER_CPR);
    getController().setTolerance(ANGLE_PID_TOLERANCE);

    speedMotor = new WPI_TalonFX(SPEED_MOTOR_PORTS[moduleNumber]);
    
    if (moduleNumber == 2 || moduleNumber == 3)
      speedMotor.setInverted(true);
      
    angleMotor = new CANSparkMax(ANGLE_MOTOR_PORTS[moduleNumber], MotorType.kBrushless);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleEncoder = new AnalogInput(ANGLE_ENCODER_PORTS[moduleNumber]);

    this.swerveDrive = swerveDrive;

    this.state = new SwerveModuleState(0.0, new Rotation2d(0.0));
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



  public void drive(SwerveModuleState state) {
    
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

    speedMotor.set(TalonFXControlMode.PercentOutput, metersPerSecond / MAX_WHEEL_SPEED);
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
    double currentPosition = getMeasurement();

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
    double halfEncoderCircle = ANGLE_ENCODER_CPR / 2;

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
    return encoderCounts * 2 * Math.PI / ANGLE_ENCODER_CPR;
  }



  /**
   * 
   * @param radians number of radians
   * @return radians converted to absolute encoder counts
   */
  public static double radiansToNative(double radians) {
    return radians / (2 * Math.PI) * ANGLE_ENCODER_CPR;
  }  
  
  
  
  /**
  * 
  * @return meters per count of talon fx encoder based on current gear
  */
  public double getMetersPerCount() {
    //     Circumference    /        Gear Ratio       / Counts Per Motor Revolution = 
    // meters per rev wheel / rev motor per rev wheel /    counts per rev motor     = Meters Per Encoder Count
    return WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIOS[swerveDrive.getCurrentGear()] / TALON_ENCODER_CPR;
  }

  

  /**
   * 
   * @return the calculated wheel speed of the module in meters per second
   */
  public double getSpeed() {
    //   Motor Velocity  * Time Conversion * Distance Per Count = 
    // Counts Per ds * 10 ds Per 1 s *  Meters Per Count  = Wheel Speed
    return speedMotor.getSelectedSensorVelocity() * 10 * getMetersPerCount();
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
    // angle encoder increases with cw movement, this conversion makes it increase with ccw movement
    // for compatibility with the radian based setpoint from kinematics class
    return -1 * angleEncoder.getAverageVoltage() + ANGLE_ENCODER_CPR;
  }



  @Override
  public void periodic() {
    super.periodic();
    // SmartDashboard.putNumber("Module " + this.moduleNumber + " setpoint", this.getSetpoint());
    // SmartDashboard.putBoolean("Module " + this.moduleNumber + " at setpoint", this.getController().atSetpoint());

    SmartDashboard.putNumber("Module " + this.moduleNumber + " measurement", getMeasurement());
    SmartDashboard.putNumber("Module " + this.moduleNumber + " gear", getCurrentGear());
    SmartDashboard.putNumber("Module " + this.moduleNumber + " speed (m/s)", getSpeed());
  }
}
