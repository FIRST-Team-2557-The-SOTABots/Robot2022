// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleFunction;

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

    // this is updated in robotPeriodic by a SmartDashboard call
    // certain constants differ between robots, this variable controls which is used
    public static boolean isCompBot = true;

    public static final double METERS_PER_INCH = 0.0254;

    public static final class Climber {
        // the frame of reference for left and right is looking in the direction the robot shoots
        public static final int LEFT_HOOK_MOTOR_PORT = 5;
        public static final int LEFT_HOOK_ENCODER_PORT = 4;
        public static final int RIGHT_HOOK_MOTOR_PORT = 2;
        public static final int RIGHT_HOOK_ENCODER_PORT = 3;
        public static final int LEFT_BOT_MAG_SENSOR_PORT = 1; 
        public static final int RIGHT_BOT_MAG_SENSOR_PORT = 2; 
        public static final int LEFT_TOP_MAG_SENSOR_PORT = 5;
        public static final int RIGHT_TOP_MAG_SENSOR_PORT = 6;
        public static final int ANGLE_MOTOR_PORT = 5;
        public static final int ANGLE_ENCODER_PORT = 0;
        public static final int SOLENOID_CHANNEL_A = 2;
        public static final int SOLENOID_CHANNEL_B = 3;
        public static final Value LOCK_VALUE = Value.kReverse;
        public static final Value UNLOCK_VALUE = Value.kForward;
        public static final boolean LEFT_HOOK_INVERTED = false; 
        public static final boolean RIGHT_HOOK_INVERTED = true; 
        public static final boolean ANGLE_HOOK_INVERTED = isCompBot ? true : true;

        // the following values are encoder positions for the left and right extending hooks
        // MIN refers to the hooks being completely lowered
        // EVEN refers to the extend hooks being at the same height as the angle hooks
        // MID refers to the position where the angle hooks would be able to pass under the extend hooks if they are attached to the bar
        // MAX refers to the position where the hooks are extended fully 
        public static final double MIN_EXTEND_ENCODER_LEFT = 0.0; 
        public static final double EVEN_EXTEND_ENCODER_LEFT = isCompBot ? 0.333 : 0.356; 
        public static final double MID_EXTEND_ENCODER_LEFT = isCompBot ? 1.720 : 1.82;
        public static final double HIGH_EXTEND_ENCODER_LEFT = isCompBot ? 6.84 : 5.69;
        public static final double MAX_EXTEND_ENCODER_LEFT = isCompBot ? 8.60 : 7.0;
        public static final double LIMIT_EXTEND_ENCODER_LEFT = isCompBot ? 12.0 : 12.0;
        public static final double MIN_EXTEND_ENCODER_RIGHT = 0.0; 
        public static final double EVEN_EXTEND_ENCODER_RIGHT = isCompBot ? 0.347 : 0.356; 
        public static final double MID_EXTEND_ENCODER_RIGHT = isCompBot ? 1.578 : 2.13;
        public static final double HIGH_EXTEND_ENCODER_RIGHT = isCompBot ? 6.96 : 6.94;
        public static final double MAX_EXTEND_ENCODER_RIGHT = isCompBot ? 7.94 : 8.7;
        public static final double LIMIT_EXTEND_ENCODER_RIGHT = isCompBot ? 12.0 : 12.0;

        // the following values are encoder positions for the angling hooks
        // MIN refers to the encoder value when the hooks are closest to the intake at the hard stop
        // MID refers to the encoder value when the hooks are even with the extending hooks
        // HIGH refers to the encoder value when the hooks are angled so that the extend hooks are on the next bar
        // HAX refers to the encoder value when the hooks are in between MID and HIGH
        // MAX refers to the encoder value when the hooks are furthest from the intake at the hard stop
        public static final int MIN_ANGLE_ENCODER = 0;
        public static final int MID_ANGLE_ENCODER = isCompBot ? 600 : 300;
        public static final int HIGH_ANGLE_ENCODER = 700;
        public static final int HAX_ANGLE_ENCODER = 1000;
        public static final int MAX_ANGLE_ENCODER = isCompBot ? 1873 : 1600;

        // soft limits for the angle hooks in encoder ticks
        public static final double ANGLE_ENCODER_HIGH_LIMIT = MAX_ANGLE_ENCODER;
        public static final double ANGLE_ENCODER_LOW_LIMIT = MIN_ANGLE_ENCODER;

        public static final double ANGLE_PID_MAX_VELOCITY = 300; // in encoder ticks per second, velocity motion profile constraint for profiled angle movement
        public static final double ANGLE_PID_MAX_ACCELERATION = 900; // in encoder ticks per second per second, acceleration motion profile constraint for profiled angle movement
        public static final double ANGLE_PID_CHANGE_KP_RANGE = 200; // in encoder ticks, error within which kp will change to the value below
        public static final double ANGLE_PID_CHANGE_KP_VALUE = 0.002; // the reduced kp value for when error is within range, since less torque is needed as the robot gets closer when doing MAX_TO_HIGH

        public static final double ANGLE_HOOKS_TO_BAR_TIMEOUT = 0.9; // time in seconds before command that moves angle hooks onto bar gives up
        public static final double SLOW_RETRACT_SPEED = -0.4; // duty cycle extend hooks retract at for reset
        public static final double RUN_TO_ANGLE_TOLERANCE = 10; // in angle encoder counts, tolerance of AngleClimbToPosition
        public static final double RUN_TO_ANGLE_SPEED = 0.2; // duty cycle angle hooks run at for AngleClimbToPosition
        public static final double RUN_TO_ANGLE_SPEED_FAST = 0.4; // duty cycle angle hooks run at for AngleClimbToPosition

        public enum SimpleExtendMovement {
            BOTTOM_TO_TOP(MAX_EXTEND_ENCODER_LEFT, MAX_EXTEND_ENCODER_RIGHT, 1.0, 0.0), 
            TOP_TO_BOTTOM(MIN_EXTEND_ENCODER_LEFT, MIN_EXTEND_ENCODER_RIGHT, 0.9, 0.0),
            BOTTOM_TO_EVEN(EVEN_EXTEND_ENCODER_LEFT, EVEN_EXTEND_ENCODER_RIGHT, 0.5, 0.1),
            EVEN_TO_MID(MID_EXTEND_ENCODER_LEFT, MID_EXTEND_ENCODER_RIGHT, 1.0, 0.1),
            MID_TO_TOP(MAX_EXTEND_ENCODER_LEFT, MAX_EXTEND_ENCODER_RIGHT, 1.0, 0.0),
            TOP_TO_HIGH(HIGH_EXTEND_ENCODER_LEFT, HIGH_EXTEND_ENCODER_RIGHT, 0.8, 0.1),
            HIGH_TO_BOTTOM(MIN_EXTEND_ENCODER_LEFT, MIN_EXTEND_ENCODER_RIGHT, 0.9, 0.0),
            BOTTOM_TO_MID(MID_EXTEND_ENCODER_LEFT, MID_EXTEND_ENCODER_RIGHT, 0.9, 0.1),
            MID_TO_BOTTOM(MIN_EXTEND_ENCODER_LEFT, MIN_EXTEND_ENCODER_RIGHT, 0.9, 0.0);

            public final double leftSetpoint;
            public final double rightSetpoint;
            public final double speed;
            public final double tolerance;

            SimpleExtendMovement(double leftSetpoint, double rightSetpoint, double speed, double tolerance) {
                this.leftSetpoint = leftSetpoint;
                this.rightSetpoint = rightSetpoint;
                this.speed = speed;
                this.tolerance = tolerance;
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
        public static final boolean[] SPEED_MOTOR_INVERTS_PRACTICE_BOT = {true, false, false, true};
        public static final boolean[] SPEED_MOTOR_INVERTS_COMP_BOT = {true, false, true, false};

        // the number that must be added to the setpoint of the module's rotation (one per module), i.e. the value of the absolute encoder when the module is straight
        public static final double[] ANGLE_ENCODER_OFFSETS_COMP_BOT = {2.781, 0.933, 2.135, 3.359}; // in encoder counts, changed offset 3 and 2
        public static final double[] ANGLE_ENCODER_OFFSETS_PRACTICE_BOT = {0.468, 4.633, 4.360, 3.919}; // in encoder counts

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

        public static final double SPEED_PID_KP = 0.001;
        public static final double SPEED_PID_KI = 0.0;
        public static final double SPEED_PID_KD = 0.0;
        public static final double SPEED_PID_TOLERANCE = 0.05; // in motor RPM

        public static final int FORWARD_CHANNEL_PORT = 6; 
        public static final int REVERSE_CHANNEL_PORT = 7;
        public static final Value LOW_GEAR_VALUE = isCompBot ? Value.kReverse : Value.kForward;
        public static final Value HIGH_GEAR_VALUE = isCompBot ? Value.kForward : Value.kReverse;

        // in counts per revolution
        public static final double TALON_ENCODER_CPR = 2048;

        // These are gear ratios, the number of rotations of driving gear per rotation of driven gear
        public static final double[] DRIVE_GEAR_RATIOS = {13.68 / 1, 6.5 / 1}; // order is low gear, high gear
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
        public static final double MAX_WHEEL_SPEED = 5.2;

        // in radians per second
        public static final double MAX_ANGULAR_SPEED = 6.28; // TODO: unoffical number
        
        public static final double COAST_DOWN_MAX_SPEED = 0.5; // speed in m/s below which shift down with low demand occurs
        public static final double COAST_DOWN_MAX_INPUT = 0.5; // input below which shift down with low demand occurs
        public static final double KICK_DOWN_MAX_SPEED = 1.0; // speed in m/s below which shift down with high demand occurs
        public static final double KICK_DOWN_MIN_INPUT = 1.0; // input above which shift down with high demand occurs
        public static final double SHIFT_UP_MIN_SPEED = 1.0; // speed in m/s above which shift up with high demand occurs
        public static final double SHIFT_UP_MIN_INPUT = 1.0; // input above which shift up with high demand occurs
        public static final double SHIFT_COOLDOWN = 1.0; // in seconds

        public static final double TARGET_SEARCH_KP = 0.075;
        public static final double TARGET_SEARCH_KI = 0.0;
        public static final double TARGET_SEARCH_KD = 0.0;
    }

    public static class Intake {
        public static final int MOTOR_PORT = 1;
        public static final boolean MOTOR_INVERTED = true;
        public static final int SOLENOID_CHANNEL_A = 0;
        public static final int SOLENOID_CHANNEL_B = 1;
        public static final Value EXTEND_VALUE = Value.kReverse;
        public static final Value RETRACT_VALUE = Value.kForward;
        public static final double SPEED = 0.9;
        public static final double EXTEND_TIME = 0.8;
    }

    public static class Shooter {
        public static final int FORWARD_CHANNEL_PORT = 4;
        public static final int REVERSE_CHANNEL_PORT = 5;
        public static final Value RAISED_VALUE = Value.kReverse;
        public static final Value LOWERED_VALUE = Value.kForward;
        public static final int MOTOR_1_PORT = 6; 
        public static final int MOTOR_2_PORT = 7;
        public static final boolean MOTOR_1_INVERTED = isCompBot ? false : true;
        public static final boolean MOTOR_2_INVERTED = !MOTOR_1_INVERTED;
        public static final double RAMP_RATE = 1; 
        public static final double GEAR_RATIO = 1.5; // 1.5 motor rotaion for every motor
        public static final double UPPER_HUB_RPM = 3900; // in motor rpm
        public static final double LOWER_HUB_RPM = 1600; // in motor rpm
        public static final double RPM_TOLERANCE = 100; // in motor rpm
        public static final double FEEDFORWARD_KS = isCompBot ?  -0.1085 : 0.0; // in volts //TODO: REDO THIS
        public static final double FEEDFORWARD_KV = isCompBot ? 0.00217 : 0.002126; // in volts
        public static final double SPEED_PID_KP = 0.0015;
        public static final double SPEED_PID_KI = 0.0; 

        public static final double SPEED_PID_KD = 0.0;
        public static final double SPEED_PID_I_ZONE = 0.0; // in RPM, max error for integral to be active
        public static final int SPEED_SAMPLE_SIZE_LIMIT = 10;
        public static final double SPOOL_RPM = UPPER_HUB_RPM * 0.66;

        public static final DoubleFunction<Double> RPM_EQUATION = (double x) -> {
            double A = 4051; 
            double B = -1.110; 
            double C = 4.653;  
            double D = -1.859;   
            double E = -0.001278;
            double F = 0.01852;  

            return 
                A + 
                B * x + 
                C * Math.pow(x, 2) + 
                D * Math.pow(x, 3) + 
                E * Math.pow(x, 4) + 
                F * Math.pow(x, 5);
        };

    }

    public static final class LimeLight {
        public static final double LIMELIGHT_CENTER = 0.0; 
        public static final double AUTOAIM_TOLERANCE = 2.0;
        public static final double MIN_TY = -8;
        public static final double MAX_TY = 11;
    }

    public static class Delivery {
        public static final int SENSOR_1_PORT = 7; // TODO fix
        public static final int SENSOR_2_PORT = 0;
        public static final int MOTOR_PORT = 4;
        public static final int SENSOR_1_LEFT_THRESHOLD = 120;
        public static final int SENSOR_1_RIGHT_THRESHOLD = 85;
        public static final int SENSOR_1_THRESHOLD = 30;
        public static final double INDEXING_SPEED = 0.5;
        public static final double SHOOTING_SPEED = 0.7; // TODO: turn this back 
        public static final boolean MOTOR_INVERTED = true;
        public static final double COOLDOWN = 0.75; // in seconds
        public static final double MAX_DELIVERY_DURATION = 0.4; // in seconds
        public static final double RETRACTED_DURATION = 0.4; // in seconds
        public static final double SENSOR_1_FILTER_TIME_CONSTANT = 0.1; // in seconds
        public static final double RAMP_RATE = 0.3;
    }

    public static class Auto {
        public static final double TRANSLATE_PID_KP = 2;
        public static final double MAX_WHEEL_SPEED = 4.0; // Swerve.MAX_WHEEL_SPEED; // in meters per second 
        public static final double MAX_WHEEL_ACCELERATION = 3.5; // Swerve.MAX_WHEEL_SPEED / 1.0; // in meters per second per second

        public static final double ANGLE_PID_KP = 3;
        public static final double MAX_ANGULAR_SPEED = Swerve.MAX_ANGULAR_SPEED + 3; // in radians per second 
        public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_SPEED / 0.1; // in radians per second per second
        
        public static final double FLYWHEEL_IDLE_SPEED = 3600; // in seconds, time intake spends out

        public static final double PATH_1_SHOOT_1_DURATION = 2;
        public static final double PATH_1_SHOOT_2_DURATION = 1.25;
        public static final double PATH_1_SHOOT_3_DURATION = 2;
        public static final double HUMAN_PLAYER_WAIT_TIME = 1.25;

        public static final double PATH_2_SHOOT_1_DURATION = 2.0;
        public static final double PATH_2_OUTTAKE_2_DURATION = 1.0;

        public static final double BACK_UP_AUTO_DURATION = 2.0; // seconds

        public static final double SHOOT_HIGH_BACK_SHOOT_DURATION = 5.0; // seconds
        public static final double SHOOT_HIGH_BACK_DRIVE_DURATION = 3.0; // seconds

        public static final double DURATION = 15.0; // seconds
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
            public static final double LEFT_STICK_Y_DEADBAND = 0.1;
            public static final double RIGHT_STICK_Y_DEADBAND = 0.1;
        }
    }

}
