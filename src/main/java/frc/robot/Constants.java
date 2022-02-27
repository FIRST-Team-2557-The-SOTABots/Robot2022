// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

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

        // all motor value arrays are in the order FL, FR, BL, BR
        public static final int[] ANGLE_MOTOR_PORTS = {8, 9, 4, 3};
        public static final boolean[] ANGLE_MOTOR_INVERTS = {false, false, false, false};

        public static final int[] SPEED_MOTOR_PORTS = {3, 2, 1, 0};
        public static final boolean[] SPEED_MOTOR_INVERTS = {true, false, true, false};

        // the number that must be added to the setpoint of the module's rotation (one per module), i.e. the value of the absolute encoder when the module is straight
        public static final double[] ANGLE_ENCODER_OFFSETS = {4.189, 0.557, 1.085, 4.089}; // in encoder counts
        public static final double ANGLE_ENCODER_CPR = 4.957; // in encoder counts
        public static final int[] ANGLE_ENCODER_PORTS = {3, 2, 1, 0};

        public static final double ANGLE_FEEDFORWARD_KS = 0.77625; // in volts
        public static final double ANGLE_FEEDFORWARD_KV = 0.39725; // in volt seconds per encoder tick

        public static final double ANGLE_PID_KP = 4;
        public static final double ANGLE_PID_KI = 0.0;
        public static final double ANGLE_PID_KD = 0.0;
        public static final double ANGLE_PID_TOLERANCE = 0.05;
        public static final double ANGLE_PID_MAX_ACCELERATION = 70.0; // in encoder ticks per second per second
        public static final double ANGLE_PID_MAX_VELOCITY = ANGLE_PID_MAX_ACCELERATION * Math.sqrt((ANGLE_ENCODER_CPR / 4) / ANGLE_PID_MAX_ACCELERATION); // in encoder ticks per second

        public static final double SPEED_FEEDFORWARD_KS = 0.0; // TODO: find real value
        public static final double SPEED_FEEDFORWARD_KV = 0.0; // TODO: find real value

        public static final double SPEED_PID_KP = 0.2; // TODO: update
        public static final double SPEED_PID_KI = 0.0;
        public static final double SPEED_PID_KD = 0.0;
        public static final double SPEED_PID_TOLERANCE = 0.05; // TODO: update

        public static final int FORWARD_CHANNEL_PORT = 6; 
        public static final int REVERSE_CHANNEL_PORT = 7;
        public static final Value LOW_GEAR_VALUE = Value.kForward;
        public static final Value HIGH_GEAR_VALUE = Value.kReverse;

        // in counts per revolution
        public static final double TALON_ENCODER_CPR = 2048;

        // These are gear ratios, the number of rotations of driving gear per rotation of driven gear
        public static final double[] DRIVE_GEAR_RATIOS = {13.6 / 1, 6.5 / 1}; // order is low gear, high gear
        public static final double TURN_GEAR_RATIO = 16 / 1;

        public static final int NUM_MODULES = 4;
        
        // in meters
        public static final double WHEEL_BASE = 23.111 * METERS_PER_INCH;
        public static final double TRACK_WIDTH = 23.111 * METERS_PER_INCH; // TODO: good?
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
        public static final double MAX_ANGULAR_SPEED = 2; // TODO: unoffical number
        
        public static final double COAST_DOWN_MAX_SPEED = 0.5; // speed in m/s below which shift down with low demand occurs
        public static final double COAST_DOWN_MAX_INPUT = 0.5; // input below which shift down with low demand occurs
        public static final double KICK_DOWN_MAX_SPEED = 1.0; // speed in m/s below which shift down with high demand occurs
        public static final double KICK_DOWN_MIN_INPUT = 1.0; // input above which shift down with high demand occurs
        public static final double SHIFT_UP_MIN_SPEED = 1.0; // speed in m/s above which shift up with high demand occurs
        public static final double SHIFT_UP_MIN_INPUT = 1.0; // input above which shift up with high demand occurs
        public static final double SHIFT_COOLDOWN = 1.0; // in seconds
    }

    public static class Intake {
        public static final int MOTOR_PORT = 1;
        public static final boolean MOTOR_INVERTED = true;
        public static final int SOLENOID_CHANNEL_A = 0;
        public static final int SOLENOID_CHANNEL_B = 1;
        public static final Value EXTEND_VALUE = Value.kReverse;
        public static final Value RETRACT_VALUE = Value.kForward;
        public static final double SPEED = 0.8;
    }

    public static class Shooter {
        public static final int FORWARD_CHANNEL_PORT = 4;
        public static final int REVERSE_CHANNEL_PORT = 5;
        public static final Value RAISED_VALUE = Value.kReverse;
        public static final Value LOWERED_VALUE = Value.kForward; // TODO: find correct directions
        public static final int MOTOR_1_PORT = 6; 
        public static final int MOTOR_2_PORT = 7;
        public static final boolean MOTOR_1_INVERTED = true;
        public static final boolean MOTOR_2_INVERTED = false;
        public static final double RAMP_RATE = 3.0;
        public static final double GEAR_RATIO = 1.5; // 1.5 motor rotaion for every motor
        public static final double UPPER_HUB_RPM = 3900; // in motor rpm
        public static final double LOWER_HUB_RPM = 1600; // in motor rpm
        public static final double RPM_TOLERANCE = 80; // in motor rpm
        public static final double FEEDFORWARD_KS = 0.03269; // in volts
        public static final double FEEDFORWARD_KV = 0.002114; // in volts
        public static final double SPEED_PID_KP = 0.00003;
        public static final double SPEED_PID_KI = 0.0;
        public static final double SPEED_PID_KD = 0.0;
        public static final double SHOOT_COOLDOWN = 1.0; // in seconds
        public static final int SPEED_SAMPLE_SIZE_LIMIT = 5;
    }

    public static class Delivery {
        public static final int SENSOR_2_PORT = 0;
        public static final int MOTOR_PORT = 4;
        public static final int SENSOR_1_THRESHOLD = 104; // 80 // TODO update
        public static final double INDEXING_SPEED = 0.45;
        public static final double SHOOTING_SPEED = 0.6;
        public static final boolean MOTOR_INVERTED = true;
        public static final double COOLDOWN = 1.0; // in seconds
        public static final double MAX_DELIVERY_DURATION = 0.2; // in seconds
        public static final double RETRACTED_DURATION = 0.4; // in seconds
    }

    public static class Control {
        public static class Driver {
            public static final int PORT = 0;
            public static final double LEFT_X_DEADBAND = 0.1;
            public static final double LEFT_Y_DEADBAND = 0.1;
            public static final double RIGHT_X_DEADBAND = 0.1;
            public static final double LEFT_TRIGGER_DEADBAND = 0.5;
            public static final double RIGHT_TRIGGER_DEADBAND = 0.5;
        }

        public static class Manipulator {
            public static final int PORT = 1;
        }
    }
}
