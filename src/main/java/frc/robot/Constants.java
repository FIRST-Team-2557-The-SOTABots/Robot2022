// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final double MID_EXTEND_ENCODER_LEFT = 2.35;
        public static final double MAX_EXTEND_ENCODER_LEFT = 9.3; 
        public static final double EXTEND_HIGH_LIMIT_LEFT = MAX_EXTEND_ENCODER_LEFT; 
        public static final double EXTEND_LOW_LIMIT_LEFT = MIN_EXTEND_ENCODER_LEFT;

        public static final double MIN_EXTEND_ENCODER_RIGHT = 0.0; 
        public static final double MID_EXTEND_ENCODER_RIGHT = 2.13;
        public static final double MAX_EXTEND_ENCODER_RIGHT = 8.7; 
        public static final double EXTEND_HIGH_LIMIT_RIGHT = MAX_EXTEND_ENCODER_RIGHT; 
        public static final double EXTEND_LOW_LIMIT_RIGHT = MIN_EXTEND_ENCODER_RIGHT;
        
        public static final double MAX_EXTEND_HOOK_LENGTH = 25 * METERS_PER_INCH; // meters // TODO update
        public static final double MIN_EXTEND_HOOK_LENGTH = 25 * METERS_PER_INCH; // meters // TODO update


        public static final int MAX_ANGLE_ENCODER = 1600; // encoder value at front hard stop 
        public static final int HIGH_ANGLE_ENCODER = 1400; // encoder value between front hard stop and extend hooks TODO find actual 
        public static final int MID_ANGLE_ENCODER = 300; // encoder value at even with extend hooks 
        public static final int MIN_ANGLE_ENCODER = 0; // encoder value at back hard stop
        public static final double ANGLE_ENCODER_HIGH_LIMIT = MAX_ANGLE_ENCODER; // TODO make better
        public static final double ANGLE_ENCODER_LOW_LIMIT = MIN_ANGLE_ENCODER;

        public static final double MAX_ANGLE = 45 * Math.PI / 180; // angle at front hard stop, in degrees TODO fix
        public static final double MIN_ANGLE = -10 * Math.PI / 180; // angle at back hard stop, in radians TODO find slope
        
        public static final double ANGLE_HOOK_LENGTH = 35 * METERS_PER_INCH; // in meters // TODO get better measurements if possible
        public static final double DISTANCE_BETWEEN_BARS = 28.5024669985 * METERS_PER_INCH; // in meters

        public static final double EXTEND_PID_TOLERANCE = 0.02;
        public static final double EXTEND_PID_OVERSHOOT = 0.3;
        
        public static final double ANGLE_PID_KP = 0.0;
        public static final double ANGLE_PID_KI = 0.0;
        public static final double ANGLE_PID_KD = 0.0;
        public static final double ANGLE_PID_TOLERANCE = 0.0;

        public static final double SLOW_RETRACT_SPEED = -0.2;

        public static final double TIMED_ANGLE_SPEED = 0.2;
        public static final double TIMED_ANGLE_DURATION = 0.3; // in seconds
        
        public enum ExtendMovement {
            BOTTOM_TO_TOP(0.65, 0.0005, 0.0, 5, 8, MAX_EXTEND_ENCODER_LEFT + EXTEND_PID_OVERSHOOT, MAX_EXTEND_ENCODER_RIGHT + EXTEND_PID_OVERSHOOT),
            TOP_TO_BOTTOM(0.65, 0.001, 0.0, 1, 2, MIN_EXTEND_ENCODER_LEFT - EXTEND_PID_OVERSHOOT, MIN_EXTEND_ENCODER_RIGHT - EXTEND_PID_OVERSHOOT),
            BOTTOM_TO_MID(0.65, 0.0005, 0.0, 5, 8, MID_EXTEND_ENCODER_LEFT, MID_EXTEND_ENCODER_RIGHT),
            MID_TO_TOP   (0.65, 0.0005, 0.0, 5, 8, MAX_EXTEND_ENCODER_LEFT, MAX_EXTEND_ENCODER_RIGHT),
            MID_TO_BOTTOM(0.65, 0.0005, 0.0, 5, 8, MIN_EXTEND_ENCODER_LEFT, MIN_EXTEND_ENCODER_RIGHT);

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
            MIN_TO_MID (0.001, 0.0005, 0,  MID_ANGLE_ENCODER, 0.0),
            MID_TO_MAX (0, 0, 0,  MAX_ANGLE_ENCODER, 0.0),
            MAX_TO_HIGH(0, 0, 0, HIGH_ANGLE_ENCODER, 0.0),
            HIGH_TO_MAX(0, 0, 0,  MAX_ANGLE_ENCODER, 0.0),
            MAX_TO_MIN (0, 0, 0,  MIN_ANGLE_ENCODER, 0.0);

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
}
