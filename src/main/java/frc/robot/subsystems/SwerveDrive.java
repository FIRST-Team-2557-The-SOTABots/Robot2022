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
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Swerve.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

public class SwerveDrive extends SubsystemBase {

  private SwerveDriveKinematics swerveDriveKinematics;
  private SwerveModule[] swerveModules = new SwerveModule[NUM_MODULES];
  private SwerveModuleState[] moduleStates = new SwerveModuleState[NUM_MODULES];

  private boolean fieldCentricActive = true;

  private AHRS gyro;
  private DoubleSolenoid shifter;
  
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
    gyro = new AHRS(SerialPort.Port.kMXP);
    gyro.reset();
    gyro.setAngleAdjustment(0.0);
    shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARD_CHANNEL_PORT, REVERSE_CHANNEL_PORT);
    shiftDown();

    odometry = new SwerveDriveOdometry(swerveDriveKinematics, new Rotation2d(getGyroAngle()), new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
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

  

  /**
   * Sets the gyro's angle
   * @param radians the angle to set in radians
   */
  public void setGyroAngle(double radians) {
    gyro.reset();
    gyro.setAngleAdjustment(-Math.toDegrees(radians));
  }



  /**
   * resets the gyro to 0 degrees
   */
  public void resetGyro() {
    gyro.setAngleAdjustment(0.0);
    gyro.reset();
  }



  /**
   * @return Pose2d object representing the robot's position and rotation
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }



  /**
   * Sets the robot's pose. Also sets the gyro angle to the angle in the pose.
   * @param state the pose to set
   */
  public void setPose(PathPlannerState state) {
    setGyroAngle(state.holonomicRotation.getRadians());
    odometry.resetPosition(
      new Pose2d(
        state.poseMeters.getX(), 
        state.poseMeters.getY(), 
        new Rotation2d(getGyroAngle())
      ), 
      new Rotation2d(getGyroAngle())
    );
  }



  /**
   * Updates the pose of the swerve drive
   */
  public void updatePose() {
    SwerveModuleState[] measuredModuleStates = new SwerveModuleState[NUM_MODULES];
    for (int i = 0; i < NUM_MODULES; i++) {
      measuredModuleStates[i] = swerveModules[i].getMeasuredState();
    }
    odometry.update(new Rotation2d(getGyroAngle()), measuredModuleStates);
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
   * @return whether field centric is active
   */
  public boolean getFieldCentricActive() {
    return this.fieldCentricActive;
  }


  /**
   * Sets the speed motors of all modules to coast mode.
   */
  public void speedMotorsCoast() {
    for (int i = 0; i < NUM_MODULES; i++) {
      swerveModules[i].speedMotorCoast();
    }
  }



  /**
   * Sets the speed motors of all modules to brake mode.
   */
  public void speedMotorsBrake() {
    for (int i = 0; i < NUM_MODULES; i++) {
      swerveModules[i].speedMotorBrake();
    }
  }



  @Override
  public void periodic() {
    // periodically update the pose of the robot
    updatePose();

    SmartDashboard.putBoolean("Field Centric Active", fieldCentricActive);
    SmartDashboard.putNumber("gyro angle", getGyroAngle());
  }
}
