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
        public static final int LEFT_MAG_SENSOR_PORT = 2; //TODO: get correct port num
        public static final int RIGHT_MAG_SENSOR_PORT = 1; //TODO: get correct port num
        public static final int ANGLE_MOTOR_PORT = 5;
        public static final int ANGLE_ENCODER_PORT = 0; // TODO: get correct port num
        public static final int SOLENOID_CHANNEL_A = 2;
        public static final int SOLENOID_CHANNEL_B = 3;

        public static final Value LOCK_VALUE = Value.kReverse;
        public static final Value UNLOCK_VALUE = Value.kForward;

        public static final boolean LEFT_HOOK_INVERTED = false; //TODO check 
        public static final boolean RIGHT_HOOK_INVERTED = true; //TODO check 
        public static final boolean ANGLE_HOOK_INVERTED = false; //TODO check 

        public static final double EXTEND_HIGH_LIMIT = 1000; // TODO make better
        public static final double EXTEND_LOW_LIMIT = 0;
        
        public static final double MIN_EXTEND_ENCODER = 0.0; //TODO get correct value
        public static final double MID_EXTEND_ENCODER = 100;
        public static final double MAX_EXTEND_ENCODER = 1000.0; //TODO get correct value
        public static final double MAX_EXTEND_HOOK_LENGTH = 25 * METERS_PER_INCH; // meters // TODO update
        public static final double MIN_EXTEND_HOOK_LENGTH = 25 * METERS_PER_INCH; // meters // TODO update

        public static final double ANGLE_ENCODER_HIGH_LIMIT = 1660; // TODO make better
        public static final double ANGLE_ENCODER_LOW_LIMIT = 10;

        public static final int MAX_ANGLE_ENCODER = 1670; // encoder value at front hard stop TODO find actual 
        public static final int HIGH_ANGLE_ENCODER = 1400; // encoder value between front hard stop and extend hooks TODO find actual 
        public static final int MID_ANGLE_ENCODER = 100; // encoder value at even with extend hooks TODO find actual 
        public static final int MIN_ANGLE_ENCODER = 0; // encoder value at back hard stop TODO find actual 
        public static final double MAX_ANGLE = 45 * Math.PI / 180; // angle at front hard stop, in degrees TODO fix
        public static final double MIN_ANGLE = -10 * Math.PI / 180; // angle at back hard stop, in radians TODO find slope
        
        public static final double ANGLE_HOOK_LENGTH = 35 * METERS_PER_INCH; // in meters // TODO get better measurements if possible
        public static final double DISTANCE_BETWEEN_BARS = 28.5024669985 * METERS_PER_INCH; // in meters

        public static final double EXTEND_PID_KP = 0.0;
        public static final double EXTEND_PID_KI = 0.0;
        public static final double EXTEND_PID_KD = 0.0;
        public static final double EXTEND_MAX_VELOCITY = 0.0;
        public static final double EXTEND_MAX_ACCELERATION = 0.0;
        
        public static final double ANGLE_PID_KP = 0.0;
        public static final double ANGLE_PID_KI = 0.0;
        public static final double ANGLE_PID_KD = 0.0;
        public static final double ANGLE_PID_TOLERANCE = 0.0;

    }
}
