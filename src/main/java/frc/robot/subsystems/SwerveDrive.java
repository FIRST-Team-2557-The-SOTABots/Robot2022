// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Swerve.*;

import com.kauailabs.navx.frc.AHRS;

public class SwerveDrive extends SubsystemBase {

  private SwerveDriveKinematics swerveDriveKinematics;
  private SwerveModule[] swerveModules = new SwerveModule[NUM_MODULES];
  private SwerveModuleState[] moduleStates = new SwerveModuleState[NUM_MODULES];

  private boolean fieldCentricActive = true;

  private AHRS gyro;
  private DoubleSolenoid shifter;
  
  private Pose2d pose;
  private SwerveDriveOdometry odometry;
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    swerveDriveKinematics = new SwerveDriveKinematics(
      FRONT_LEFT_MODULE_POSITION,
      FRONT_RIGHT_MODULE_POSITION,
      BACK_LEFT_MODULE_POSITION,
      BACK_RIGHT_MODULE_POSITION
    );
    for (int i = 0; i < NUM_MODULES; i++) {
      swerveModules[i] = new SwerveModule(i, this);
    }
    for (int i = 0; i < NUM_MODULES; i++) {
      moduleStates[i] = new SwerveModuleState(0.0, new Rotation2d(0.0));
    }

    gyro = new AHRS(Port.kMXP);
    gyro.reset();
    shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARD_CHANNEL_PORT, REVERSE_CHANNEL_PORT);
    shiftDown();

    // construct swerve pose with values from constants as starting point
    pose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    odometry = new SwerveDriveOdometry(swerveDriveKinematics, new Rotation2d(getGyroAngle()), pose);
  }



  /**
   * Shifts the drivetrain automatically based on configured parameters, returning whether a shift occured.
   * @param xThrottle the input for the robot's forward velocity
   * @param yThrottle the input for the robot's left velocity
   * @return whether a shift occured
   */
  public boolean autoShift(double xThrottle, double yThrottle) {
    ChassisSpeeds chassisSpeeds = getMeasuredChassisSpeeds();
    double drivetrainSpeed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    double inputMagnitude = Math.hypot(xThrottle, yThrottle);
    double accelerationMagnitude = Math.hypot((double) gyro.getWorldLinearAccelX(), (double) gyro.getWorldLinearAccelY());

    if (drivetrainSpeed < COAST_DOWN_MAX_SPEED && inputMagnitude < COAST_DOWN_MAX_INPUT) {
      shiftDown();
      return true;
    }
    if (drivetrainSpeed < KICK_DOWN_MAX_SPEED && inputMagnitude > KICK_DOWN_MIN_INPUT) {
      shiftDown();
      return true;
    }
    if (drivetrainSpeed > SHIFT_UP_MIN_SPEED && inputMagnitude > SHIFT_UP_MIN_INPUT) {
      shiftUp();
      return true;
    }
    return false;
  }



  /**
   * This method makes the swerve drivetrain drive based on 
   * @param xVelocity the forward velocity of the robot in meters per second
   * @param yVelocity the left velocity of the robot in meters per second
   * @param angularVelocity the counter-clockwise angular velocity of the robot in radians per second
   */
  public void drive(double xVelocity, double yVelocity, double angularVelocity) {    
    // create chassis input state, converting from field relative if needed
    ChassisSpeeds input = fieldCentricActive ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, new Rotation2d(getGyroAngle())) :
      new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);

    // convert chassis input into wheel input, normalizing wheel speeds, then set each wheel
    moduleStates = swerveDriveKinematics.toSwerveModuleStates(input);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_WHEEL_SPEED);
    drive(moduleStates);
  }



  /**
   * 
   * @param moduleStates the angle and speed each module should attain
   */
  public void drive(SwerveModuleState[] states) {
    moduleStates = states;
    for (int i = 0; i < NUM_MODULES; i++) {
      swerveModules[i].drive(states[i]);
    }
  }



  /**
   * Shifts to high gear
   */
  public void shiftUp() {
    shifter.set(HIGH_GEAR_VALUE);
  }



  /**
   * Shifts to low gear
   */
  public void shiftDown() {
    shifter.set(LOW_GEAR_VALUE);
  }



  /**
   * @return gyro angle but in radians
   */
  public double getGyroAngle() {
    return -Math.toRadians(gyro.getAngle());
  }

  

  public void setGyroAngle(double radians) {
    gyro.reset();
    gyro.setAngleAdjustment(-Math.toDegrees(radians));
  }



  /**
   * resets the gyro to 0 degrees
   */
  public void resetGyro() {
    gyro.reset();
  }



  /**
   * @return Pose2d object representing the robot's position and rotation
   */
  public Pose2d getSwervePose() {
    return pose;
  }



  /**
   * Updates the pose of the swerve drive
   */
  public void updatePose() {
    pose = odometry.update(new Rotation2d(getGyroAngle()), moduleStates);
  }



  /**
   * 
   * @return the swerve drive's kinematics object
   */
  public SwerveDriveKinematics getKinematics() {
      return swerveDriveKinematics;
  }


  /**
   * 
   * @return the current gear. 0 is low, 1 is high
   */
  public int getCurrentGear() {
      return shifter.get() == HIGH_GEAR_VALUE ? 1 : 0;
  }



  /**
   * 
   * @return the state of the chassis as measured by encoders, not the set state
   */
  public ChassisSpeeds getMeasuredChassisSpeeds() {
    SwerveModuleState[] measuredModuleStates = new SwerveModuleState[NUM_MODULES];
    for (int i = 0; i < NUM_MODULES; i++)
      measuredModuleStates[i] = swerveModules[i].getMeasuredState();
    return swerveDriveKinematics.toChassisSpeeds(measuredModuleStates);
  }



  /**
   * 
   * @param fieldCentricActive whether to activate field centric mode
   */
  public void setFieldCentricActive(boolean fieldCentricActive) {
    this.fieldCentricActive = fieldCentricActive;
  }



  /**
   * 
   * @return whether field 
   */
  public boolean getFieldCentricActive() {
    return this.fieldCentricActive;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose();
    SmartDashboard.putBoolean("Field Centric Active", fieldCentricActive);
  }
}
