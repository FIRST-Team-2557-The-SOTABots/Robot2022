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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Swerve.*;

public class SwerveModule extends SubsystemBase {
  
  private int moduleNumber;
  
  private WPI_TalonFX speedMotor;
  private CANSparkMax angleMotor;
  private AnalogInput angleEncoder;
  
  private SwerveDrive swerveDrive;

  private SimpleMotorFeedforward angleFF;
  private ProfiledPIDController anglePID;
  private SimpleMotorFeedforward speedFF;
  private PIDController speedPID;

  /** Creates a new SwerveModule. */
  public SwerveModule(int moduleNumber, SwerveDrive swerveDrive) {

    this.moduleNumber = moduleNumber;

    speedMotor = new WPI_TalonFX(SPEED_MOTOR_PORTS[moduleNumber]);
    speedMotor.configFactoryDefault();
    speedMotor.setInverted(SPEED_MOTOR_INVERTS[moduleNumber]);
    angleMotor = new CANSparkMax(ANGLE_MOTOR_PORTS[moduleNumber], MotorType.kBrushless);
    angleMotor.restoreFactoryDefaults();
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setInverted(ANGLE_MOTOR_INVERTS[moduleNumber]);
    angleEncoder = new AnalogInput(ANGLE_ENCODER_PORTS[moduleNumber]);

    this.swerveDrive = swerveDrive;

    // NOTE: angle PID deals in native absolute encoder units, where counterclockwise module rotation (from above) is positive
    // speed PID deals in native integrated quadrature encoder units
    angleFF = new SimpleMotorFeedforward(ANGLE_FEEDFORWARD_KS, ANGLE_FEEDFORWARD_KV);
    anglePID = new ProfiledPIDController(
      ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD, 
      new TrapezoidProfile.Constraints(ANGLE_PID_MAX_VELOCITY, ANGLE_PID_MAX_ACCELERATION));
    anglePID.enableContinuousInput(0, ANGLE_ENCODER_CPR);
    anglePID.setTolerance(ANGLE_PID_TOLERANCE);
    speedFF = new SimpleMotorFeedforward(SPEED_FEEDFORWARD_KS, SPEED_FEEDFORWARD_KV);
    speedPID = new PIDController(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD);
  }



  /**
   * 
   * @param state the speed and rotation the module should track
   */
  public void drive(SwerveModuleState state) {
    // optimize state so that module never turns more than 90 degrees
    state = SwerveModuleState.optimize(state, new Rotation2d(nativeToRadians(getAngle())));

    double angleSetpointNative = radiansToNative(state.angle.getRadians()) + ANGLE_ENCODER_OFFSETS[moduleNumber];
    double anglePIDOutput = anglePID.calculate(getAngle(), angleSetpointNative);
    double angleFFOutput = angleFF.calculate(anglePID.getSetpoint().velocity);

    angleMotor.setVoltage(angleFFOutput + anglePIDOutput);

    double speedSetpointNative = metersPerSecondToNative(state.speedMetersPerSecond);
    double speedPIDOutput = speedPID.calculate(speedMotor.getSelectedSensorVelocity(), speedSetpointNative);
    double speedFFOutput = speedFF.calculate(speedSetpointNative);

    // TODO: speed should be 0 unless module is at its angle setpoint
    // speedMotor.setVoltage(speedFFOutput + speedPIDOutput); // TODO: enable
    speedMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / MAX_WHEEL_SPEED); // TODO: delete after testing
  }



  /**
   * @param encoderCounts number of encoder counts to convert
   * @return absolute encoder counts converted to radians
   */
  public static double nativeToRadians(double encoderCounts) {
    return encoderCounts * 2 * Math.PI / ANGLE_ENCODER_CPR;
  }



  /**
   * 
   * @param radians number of radians to convert
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
    return WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIOS[getCurrentGear()] / TALON_ENCODER_CPR;
  }



  /**
   * 
   * @param metersPerSecond the speed in meters per second to be converted
   * @return the equivalent speed motor encoder velocity in counts per 100 ms
   */
  public double metersPerSecondToNative(double metersPerSecond) {
    return metersPerSecond / getMetersPerCount() / 10;
  }

  

  /**
   * 
   * @param encoderVelocity the velocity to be converted in encoder counts per decisecond
   * @return the converted speed of the wheel in meters per second
   */
  public double nativeToMetersPerSecond(double encoderVelocity) {
    return speedMotor.getSelectedSensorVelocity() * 10 * getMetersPerCount();
  }



  /**
   * 
   * @return the current gear of the module
   */
  public int getCurrentGear() {
      return swerveDrive.getCurrentGear();
  }



  /**
   * 
   * @return the angle of the module in native units
   */
  public double getAngle() {
    // Return the process variable measurement here
    // angle encoder increases with cw movement, this conversion makes it increase with ccw movement
    // for compatibility with the radian based setpoint from kinematics class
    return -1 * angleEncoder.getAverageVoltage() + ANGLE_ENCODER_CPR;
  }



  /**
   * 
   * @return the speed of the module in meters per second
   */
  public double getSpeed() {
    return nativeToMetersPerSecond(speedMotor.getSelectedSensorVelocity());
  }



  /**
   * To accurately track the behavior of the drivetrain, knowing the actual state of the module is necessary since set state
   * often varies from actual state.
   * @return the measured state of the swerve module (not set state!)
   */
  public SwerveModuleState getMeasuredState() {
    SwerveModuleState state = new SwerveModuleState(getSpeed(), new Rotation2d(nativeToRadians(getAngle())));
    // TODO: uncomment if this is necessary
    // // state should have positive wheel speed, flip direction if flipping wheel speed is necessary  
    // if (state.speedMetersPerSecond < 0.0) {
    //   state.speedMetersPerSecond *= -1;
    //   state.angle = state.angle.plus(new Rotation2d(Math.PI));
    // }
    return state;
  }



  /**
   * 
   * @return whether the module is angled to the set angle
   */
  public boolean atAngleSetpoint() {
    return anglePID.atGoal();
  }



  @Override
  public void periodic() {
  }
}
