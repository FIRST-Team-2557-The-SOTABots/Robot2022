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

    public static final class Climber {
        public static final int LEFT_HOOK_MOTOR_PORT = 5;
        public static final int LEFT_HOOK_ENCODER_PORT = 4;
        public static final int RIGHT_HOOK_MOTOR_PORT = 2;
        public static final int RIGHT_HOOK_ENCODER_PORT = 3;
        public static final int LEFT_BOT_MAG_SENSOR_PORT = 1; 
        public static final int RIGHT_BOT_MAG_SENSOR_PORT = 2; 
        public static final int LEFT_TOP_MAG_SENSOR_PORT = 5; // TODO get correct port num
        public static final int RIGHT_TOP_MAG_SENSOR_PORT = 6; // TODO get correct port num
        public static final int ANGLE_MOTOR_PORT = 5;
        public static final int ANGLE_ENCODER_PORT = 0; // TODO: get correct port num
        public static final int SOLENOID_CHANNEL_A = 2;
        public static final int SOLENOID_CHANNEL_B = 3;

        public static final Value LOCK_VALUE = Value.kReverse;
        public static final Value UNLOCK_VALUE = Value.kForward;

        public static final boolean LEFT_HOOK_INVERTED = false; 
        public static final boolean RIGHT_HOOK_INVERTED = true; 
        public static final boolean ANGLE_HOOK_INVERTED = true; //TODO check 

        public static final double MIN_EXTEND_ENCODER_LEFT = 0.0; 
        public static final double EVEN_EXTEND_ENCODER_LEFT = 0.356; 
        public static final double MID_EXTEND_ENCODER_LEFT = 1.82;
        public static final double HIGH_EXTEND_ENCODER_LEFT = 5.69;
        public static final double MAX_EXTEND_ENCODER_LEFT = 7.0; 
        public static final double EXTEND_HIGH_LIMIT_LEFT = MAX_EXTEND_ENCODER_LEFT; 
        public static final double EXTEND_LOW_LIMIT_LEFT = MIN_EXTEND_ENCODER_LEFT;

        public static final double MIN_EXTEND_ENCODER_RIGHT = 0.0; 
        public static final double EVEN_EXTEND_ENCODER_RIGHT = 0.356; 
        public static final double MID_EXTEND_ENCODER_RIGHT = 2.13;
        public static final double HIGH_EXTEND_ENCODER_RIGHT = 6.94;
        public static final double MAX_EXTEND_ENCODER_RIGHT = 8.7; 
        public static final double EXTEND_HIGH_LIMIT_RIGHT = MAX_EXTEND_ENCODER_RIGHT; 
        public static final double EXTEND_LOW_LIMIT_RIGHT = MIN_EXTEND_ENCODER_RIGHT;
        
        public static final double MAX_EXTEND_HOOK_LENGTH = 25 * METERS_PER_INCH; // meters // TODO update
        public static final double MIN_EXTEND_HOOK_LENGTH = 25 * METERS_PER_INCH; // meters // TODO update


        public static final int MAX_ANGLE_ENCODER = 1600; // encoder value at front hard stop 
        public static final int HIGH_ANGLE_ENCODER = 700; // 1250 // encoder value between front hard stop and extend hooks TODO find actual 
        public static final int MID_ANGLE_ENCODER = 300; // encoder value at even with extend hooks 
        public static final int MIN_ANGLE_ENCODER = 0; // encoder value at back hard stop
        public static final double ANGLE_ENCODER_HIGH_LIMIT = MAX_ANGLE_ENCODER;
        public static final double ANGLE_ENCODER_LOW_LIMIT = MIN_ANGLE_ENCODER;

        public static final double MAX_ANGLE = 45 * Math.PI / 180; // angle at front hard stop, in degrees TODO fix
        public static final double MIN_ANGLE = -10 * Math.PI / 180; // angle at back hard stop, in radians TODO find slope
        
        public static final double ANGLE_HOOK_LENGTH = 35 * METERS_PER_INCH; // in meters // TODO get better measurements if possible
        public static final double DISTANCE_BETWEEN_BARS = 28.5024669985 * METERS_PER_INCH; // in meters

        public static final double EXTEND_PID_TOLERANCE = 0.1; // 0.08;
        public static final double EXTEND_PID_OVERSHOOT = 0.3; // aim to extend slightly beyond max value to help reach the setpoint
        
        public static final double ANGLE_PID_TOLERANCE = 0.0;
        public static final double ANGLE_PID_MAX_VELOCITY = 300;
        public static final double ANGLE_PID_MAX_ACCELERATION = 900;
        public static final double ANGLE_PID_CHANGE_KP_RANGE = 200;
        public static final double ANGLE_PID_CHANGE_KP_VALUE = 0.002;

        public static final double ANGLE_PID_PAUSE = 1;
        public static final double ANGLED_EXTEND_TIMEOUT = 3;
        public static final double SLOW_RETRACT_SPEED = -0.4;
        public static final double TIMED_ANGLE_SPEED = 0.2;
        public static final double TIMED_ANGLE_DURATION = 0.4; // in seconds
        public static final double TIMED_ANGLE_DURATION_2 = 0.4; // in seconds
        
        public enum ExtendMovement {
            BOTTOM_TO_TOP(0.65, 0.0005, 0.0, 6, 6, MAX_EXTEND_ENCODER_LEFT + EXTEND_PID_OVERSHOOT, MAX_EXTEND_ENCODER_RIGHT + EXTEND_PID_OVERSHOOT),
            TOP_TO_BOTTOM(0.7, 0.35, 0.0, 2, 2, MIN_EXTEND_ENCODER_LEFT - EXTEND_PID_OVERSHOOT, MIN_EXTEND_ENCODER_RIGHT - EXTEND_PID_OVERSHOOT),
            HANG_BOTTOM(0.65, 0.08, 0.0, 1, 2, MIN_EXTEND_ENCODER_LEFT - EXTEND_PID_OVERSHOOT, MIN_EXTEND_ENCODER_RIGHT - EXTEND_PID_OVERSHOOT),
            BOTTOM_TO_EVEN(0.65, 0.1, 0.0, 1, 2, EVEN_EXTEND_ENCODER_LEFT, EVEN_EXTEND_ENCODER_RIGHT),
            EVEN_TO_MID(0.7, 0.005, 0.0, 5, 8, MID_EXTEND_ENCODER_LEFT, MID_EXTEND_ENCODER_RIGHT),
            MID_TO_TOP(0.65, 0.005, 0.0, 5, 8, MAX_EXTEND_ENCODER_LEFT + EXTEND_PID_OVERSHOOT, MAX_EXTEND_ENCODER_RIGHT + EXTEND_PID_OVERSHOOT),
            TOP_TO_HIGH(0.65, 0.005, 0.0, 2, 2, HIGH_EXTEND_ENCODER_LEFT, HIGH_EXTEND_ENCODER_RIGHT),
            HIGH_TO_BOTTOM(0.7, 0.2, 0.0, 2, 2, MIN_EXTEND_ENCODER_LEFT - EXTEND_PID_OVERSHOOT, MIN_EXTEND_ENCODER_RIGHT - EXTEND_PID_OVERSHOOT),
            BOTTOM_TO_MID(0.65, 0.1, 0.0, 1, 2, MID_EXTEND_ENCODER_LEFT, MID_EXTEND_ENCODER_RIGHT),
            MID_TO_BOTTOM(0.7, 0.2, 0.0, 2, 2, MIN_EXTEND_ENCODER_LEFT - EXTEND_PID_OVERSHOOT, MIN_EXTEND_ENCODER_RIGHT - EXTEND_PID_OVERSHOOT);

            public final double kp;
            public final double ki;
            public final double kd;
            public final double maxVelocity;
            public final double maxAcceleration;
            public final double leftGoal;
            public final double rightGoal;

            ExtendMovement(double kp, double ki, double kd, double maxVelocity, double maxAcceleration, double leftGoal, double rightGoal) {
                this.kp = kp;
                this.ki = ki;
                this.kd = kd;
                this.maxVelocity = maxVelocity; 
                this.maxAcceleration = maxAcceleration;
                this.leftGoal = leftGoal;
                this.rightGoal = rightGoal;
            }
        }

        public enum AngleMovement {
            MIN_TO_MID(0.001, 0.0005, 0, MID_ANGLE_ENCODER, 0.0),
            MID_TO_MAX(0.00008, 0.0005, 0, MAX_ANGLE_ENCODER, 50.0),
            MAX_TO_HIGH(0.004, 0.005, 0, HIGH_ANGLE_ENCODER, 50.0),
            HOLD_HIGH(0.002, 0.005, 0, HIGH_ANGLE_ENCODER, 0.0),
            HIGH_TO_MAX(0, 0, 0, MAX_ANGLE_ENCODER, 0.0),
            MAX_TO_HIGH_NO_LOAD(0.00007, 0.0005, 0, HIGH_ANGLE_ENCODER, 250.0),
            HIGH_TO_MIN(0.00003, 0.0005, 0, MIN_ANGLE_ENCODER, 50.0);

            public final double kp;
            public final double ki;
            public final double kd;
            public final double setpoint;
            public final double tolerance;

            AngleMovement(double kp, double ki, double kd, double setpoint, double tolerance) {
                this.kp = kp;
                this.ki = ki;
                this.kd = kd;
                this.setpoint = setpoint;
                this.tolerance = tolerance;
            }
        }
    }

    public static class Swerve {

        // all motor value arrays are in the order FL, FR, BL, BR
        public static final int[] ANGLE_MOTOR_PORTS = {8, 9, 4, 3};
        public static final boolean[] ANGLE_MOTOR_INVERTS = {false, false, false, false};

        public static final int[] SPEED_MOTOR_PORTS = {3, 2, 1, 0};
        public static final boolean[] SPEED_MOTOR_INVERTS = {true, false, true, false};

        // the number that must be added to the setpoint of the module's rotation (one per module), i.e. the value of the absolute encoder when the module is straight
        public static final double[] ANGLE_ENCODER_OFFSETS = {2.251, 2.876, 4.121, 4.902}; // in encoder counts
        public static final double ANGLE_ENCODER_CPR = 5.0; // in encoder counts
        public static final int[] ANGLE_ENCODER_PORTS = {3, 2, 1, 0};

        public static final double ANGLE_FEEDFORWARD_KS = 0.77625; // in volts
        public static final double ANGLE_FEEDFORWARD_KV = 0.39725; // in volt seconds per encoder tick

        public static final double ANGLE_PID_KP = 4;
        public static final double ANGLE_PID_KI = 0.0;
        public static final double ANGLE_PID_KD = 0.0;
        public static final double ANGLE_PID_TOLERANCE = 0.000;
        public static final double ANGLE_PID_MAX_ACCELERATION = 70.0; // in encoder ticks per second per second
        public static final double ANGLE_PID_MAX_VELOCITY = ANGLE_PID_MAX_ACCELERATION * Math.sqrt((ANGLE_ENCODER_CPR / 4) / ANGLE_PID_MAX_ACCELERATION); // in encoder ticks per second

        public static final double[] SPEED_FEEDFORWARD_KS = {0.6284, 0.6284, 0.6284, 0.6284}; //0.4000}; // in volts
        public static final double[] SPEED_FEEDFORWARD_KV = {0.0005339, 0.0005339, 0.0005339, 0.0005339}; //0.0005851}; // in volt seconds per encoder tick

        public static final double SPEED_PID_KP = 0.001; // TODO: update
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
        public static final double MAX_WHEEL_SPEED = 4; // TODO: unofficial number

        // in radians per second
        public static final double MAX_ANGULAR_SPEED = 4; // TODO: unoffical number
        
        public static final double COAST_DOWN_MAX_SPEED = 0.5; // speed in m/s below which shift down with low demand occurs
        public static final double COAST_DOWN_MAX_INPUT = 0.5; // input below which shift down with low demand occurs
        public static final double KICK_DOWN_MAX_SPEED = 1.0; // speed in m/s below which shift down with high demand occurs
        public static final double KICK_DOWN_MIN_INPUT = 1.0; // input above which shift down with high demand occurs
        public static final double SHIFT_UP_MIN_SPEED = 1.0; // speed in m/s above which shift up with high demand occurs
        public static final double SHIFT_UP_MIN_INPUT = 1.0; // input above which shift up with high demand occurs
        public static final double SHIFT_COOLDOWN = 1.0; // in seconds

        public static final double TARGET_SEARCH_SPEED = 0.5; // TODO: unoffical number
    }

    public static class Intake {
        public static final int MOTOR_PORT = 1;
        public static final boolean MOTOR_INVERTED = true;
        public static final int SOLENOID_CHANNEL_A = 0;
        public static final int SOLENOID_CHANNEL_B = 1;
        public static final Value EXTEND_VALUE = Value.kReverse;
        public static final Value RETRACT_VALUE = Value.kForward;
        public static final double SPEED = 1;
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
        public static final double RAMP_RATE = 1.5;
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
        public static final double RPM_PER_DISTANCE = 500; // in cm TODO: get real number
    }

    public static final class LimeLight {
        public static final double LIMELIGHT_HEIGHT = 120;
        public static final double LIMELIGHT_MOUNT_ANGLE = 20;
        public static final double TARGET_HEIGHT = 150;
        public static final double LIMELIGHT_OFFSET = 0; 
    }

    public static class Delivery {
        public static final int SENSOR_1_LEFT_PORT = 0;
        public static final int SENSOR_1_RIGHT_PORT = 1;
        public static final int SENSOR_2_PORT = 0;
        public static final int MOTOR_PORT = 4;
        public static final int SENSOR_1_LEFT_THRESHOLD = 130;
        public static final int SENSOR_1_RIGHT_THRESHOLD = 100;
        public static final double INDEXING_SPEED = 0.45;
        public static final double SHOOTING_SPEED = 0.6;
        public static final boolean MOTOR_INVERTED = true;
        public static final double COOLDOWN = 1.0; // in seconds
        public static final double MAX_DELIVERY_DURATION = 0.4; // in seconds
        public static final double RETRACTED_DURATION = 0.4; // in seconds
    }

    public static class Auto {
        public static final double X_PID_KP = 0.0;
        public static final double Y_PID_KP = 0.0;
        public static final double MAX_WHEEL_SPEED = 1.0; // in meters per second 
        public static final double MAX_WHEEL_ACCELERATION = 1.0; // in meters per second per second

        public static final double ANGLE_PID_KP = 0.0;
        public static final double MAX_ANGULAR_SPEED = Math.PI * 0.5; // in radians per second 
        public static final double MAX_ANGULAR_ACCELERATION = Math.PI * 0.5; // in radians per second per second
        
        public static final double BACK_UP_AUTO_DURATION = 2.0; // seconds

        public static final double SHOOT_HIGH_BACK_SHOOT_DURATION = 5.0; // seconds
        public static final double SHOOT_HIGH_BACK_DRIVE_DURATION = 3.0; // seconds
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
            public static final double LEFT_TRIGGER_DEADBAND = 0.5;
            public static final double RIGHT_TRIGGER_DEADBAND = 0.5;
            public static final double LEFT_STICK_Y_DEADBAND = 0.5;
            public static final double RIGHT_STICK_Y_DEADBAND = 0.5;
        }
    }

}
