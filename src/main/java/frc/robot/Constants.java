// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final double METERS_PER_INCH = 0.0254;

    public static class Swerve {

        public static final double ANGLE_PID_KP = 0.2;
        public static final double ANGLE_PID_KI = 0.0;
        public static final double ANGLE_PID_KD = 0.0;
        public static final double ANGLE_PID_TOLERANCE = 0.05;

        // the motor ports should be in the order FL, FR, BL, BR
        public static final int[] SPEED_MOTOR_PORTS = {0, 1, 2, 3};
        public static final int[] ANGLE_MOTOR_PORTS = {4, 5, 6, 7};
        public static final int[] ANGLE_ENCODER_PORTS = {0, 1, 2, 3};
        public static final int FORWARD_CHANNEL_PORT = 0; 
        public static final int REVERSE_CHANNEL_PORT = 1;
        public static final Value LOW_GEAR_VALUE = Value.kReverse; // FIXME
        public static final Value HIGH_GEAR_VALUE = Value.kForward; // FIXME
        public static final Port GYRO_PORTS = Port.kMXP;

        // in encoder counts
        // the number that must be added to the setpoint of the module's rotation (one per module)
        // i.e. the value of the absolute encoder when the module is straight
        public static final double[] ANGLE_ENCODER_OFFSETS = {4.442, 1.188, 0.490, 1.768};

        // in encoder counts per revolution
        // CCW from above is positive direction
        public static final double ANGLE_ENCODER_CPR = 4.955;

        // in counts per revolution
        public static final double TALON_ENCODER_CPR = 2048;

        // These are gear ratios, the number of rotations of driving gear per rotation of driven gear
        public static final double[] DRIVE_GEAR_RATIOS = {13.6 / 1, 6.5 / 1}; // order is low gear, high gear
        public static final double TURN_GEAR_RATIO = 16 / 1;

        public static final int NUM_MODULES = 4;
        
        // in meters
        public static final double WHEEL_BASE = 23.111 * METERS_PER_INCH;
        public static final double TRACK_WIDTH = 23.111 * METERS_PER_INCH;
        public static final double WHEEL_DIAMETER = 4.0 * METERS_PER_INCH;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        // translation 2d considers the front of the robot as the positive x direction
        // and the left of the robot as the positive y direction
        public static final Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d BACK_LEFT_MODULE_POSITION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d BACK_RIGHT_MODULE_POSITION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

        // in meters per second
        public static final double MAX_WHEEL_SPEED = 1; // TODO: unofficial number

        // in radians per second
        public static final double MAX_ANGULAR_SPEED = 1; // TODO: unoffical number
    }

    public static class Control {
        public static class Driver {
            public static final int PORT = 0;
            public static final double LEFT_X_DEADBAND = 0.1;
            public static final double LEFT_Y_DEADBAND = 0.1;
            public static final double RIGHT_X_DEADBAND = 0.1;
            public static final double RIGHT_Y_DEADBAND = 0.1;
        }
    }
}
