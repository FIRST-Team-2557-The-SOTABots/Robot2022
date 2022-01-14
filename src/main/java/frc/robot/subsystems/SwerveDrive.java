// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PCM;

import static frc.robot.Constants.Swerve.*;

public class SwerveDrive extends SubsystemBase {

  private SwerveDriveKinematics mSwerveDriveKinematics;
  private SwerveModule[] swerveModules;
  private SwerveModuleState[] moduleStates;
  private Pose2d pose;
  private SwerveDriveOdometry odometry;

  private boolean fieldCentricActive = true;
  private int currentGear;

  // private AHRS gyro;
  private DoubleSolenoid shifter;
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    mSwerveDriveKinematics = new SwerveDriveKinematics(
      ModulePositions.FRONT_LEFT,
      ModulePositions.FRONT_RIGHT,
      ModulePositions.BACK_LEFT,
      ModulePositions.BACK_RIGHT
    );

    swerveModules = new SwerveModule[NUM_MODULES];
    
    for (int i = 0; i < NUM_MODULES; i++) {
      swerveModules[i] = new SwerveModule(i, this);
      
      swerveModules[i].enable();
    }

    // the double solenoid is connected to all module's shifters
    shifter = new DoubleSolenoid(PCM.PORT, PneumaticsModuleType.CTREPCM, Ports.FORWARD_CHANNEL, Ports.REVERSE_CHANNEL);
    shiftDown();

    // gyro = new AHRS(Ports.GYRO);
    // gyro.reset();
    
    // construct swerve pose with values from constants as starting point
    // mSwervePose = new Pose2d(INITAL_POSE.getX(), INITAL_POSE.getY(), INITAL_POSE.getRotation());
    // mSwerveDriveOdometry = new SwerveDriveOdometry(mSwerveDriveKinematics, new Rotation2d(getGyroAngle()), mSwervePose);
  }



  /**
   * This method makes the swerve drivetrain drive based on 
   * @param forward the forward velocity of the robot
   * @param strafe the left velocity of the robot
   * @param rotate the counter-clockwise angular velocity of the robot
   */
  public void drive(double forward, double strafe, double rotate) {
    // transform the input forward strafe and rotate for field centric drive
    // if (fieldCentricActive) {
    //   // convert the gyro angle from clockwise degrees to counterclockwise radians
    //   double gyroAngle = getGyroAngle();

    //   double tempForward = forward * Math.cos(gyroAngle) + strafe * Math.sin(gyroAngle);
    //   strafe = -forward * Math.sin(gyroAngle) + strafe * Math.cos(gyroAngle);
    //   forward = tempForward;
    // }
    
    // scale the max speeds by factors of the corresponding parameters
    ChassisSpeeds input = new ChassisSpeeds(
      forward * MAX_WHEEL_SPEED,
      strafe * MAX_WHEEL_SPEED, 
      rotate * MAX_ANGULAR_SPEED
    );

    // use the kinematics class to turn robot velocities into wheel speeds and angles
    // also normalize the speeds so they do not exceed the current max wheel speed
    moduleStates = mSwerveDriveKinematics.toSwerveModuleStates(input);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_WHEEL_SPEED);

    for (int i = 0; i < NUM_MODULES; i++) {
      // make the setpoint of the rotation pid the encoder amount of radians the kinematics class determined 
      // offset this amount of encoder counts by the offset corresponding to this swerve module
      // likewise set the speed equal to the value determined by the kinematics class
      double setpoint = SwerveModule.radiansToNative(moduleStates[i].angle.getRadians()) + AngleEncoder.OFFSETS[i];
      double speed = moduleStates[i].speedMetersPerSecond;

      swerveModules[i].drive(setpoint, speed);
    }
  }



  /**
   * 
   * @param moduleStates the angle and speed each module should attain
   */
  public void drive(SwerveModuleState[] states) {
    moduleStates = states;
    for (int i = 0; i < NUM_MODULES; i++) {
      double setpoint = SwerveModule.radiansToNative(moduleStates[i].angle.getRadians()) + AngleEncoder.OFFSETS[i];
      double speed = moduleStates[i].speedMetersPerSecond;

      swerveModules[i].drive(setpoint, speed);
    }
  }



  /**
   * Shifts to high gear
   */
  public void shiftUp() {
    currentGear = 1;
    shifter.set(Ports.HIGH_GEAR_VALUE);
  }



  /**
   * Shifts to low gear
   */
  public void shiftDown() {
    currentGear = 0;
    shifter.set(Ports.LOW_GEAR_VALUE);
  }



  // /**
  //  * @return gyro angle but in radians
  //  */
  // public double getGyroAngle() {
  //   return -Math.toRadians(gyro.getAngle());
  // }



  // /**
  //  * resets the gyro to 0 degrees
  //  */
  // public void resetGyro() {
  //   gyro.reset();
  // }



  // /**
  //  * @return Pose2d object representing the robot's position and rotation
  //  */
  // public Pose2d getSwervePose() {
  //   return mSwervePose;
  // }



  // /**
  //  * Updates the pose of the swerve drive
  //  */
  // public void updatePose() {

  // }



  /**
   * 
   * @return the current gear. 0 is low, 1 is high
   */
  public int getCurrentGear() {
      return currentGear;
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
    // TODO: add in pose updater
  }
}
