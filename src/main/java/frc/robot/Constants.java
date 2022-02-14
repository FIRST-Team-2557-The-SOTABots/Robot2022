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
        public static final int LEFT_HOOK_MOTOR_PORT = 2;
        public static final int RIGHT_HOOK_MOTOR_PORT = 5;
        public static final int ANGLE_MOTOR_PORT = 5;
        public static final int FORWARD_CHANNEL = 2;
        public static final int REVERSE_CHANNEL = 3;
        public static final double HOOK_START_POSITION = 0; //TODO get correct value
        public static final int ANGLE_HOOK_START_POSITION = 0; //TODO get correct value
        public static final Value LOCK_VALUE = Value.kReverse;
        public static final Value UNLOCK_VALUE = Value.kForward;
        public static final boolean LEFT_HOOK_INVERTED = false; //TODO check 
        public static final boolean RIGHT_HOOK_INVERTED = false; //TODO check 
        public static final boolean ANGLE_HOOK_INVERTED = false; //TODO check 
        public static final double EXTEND_HIGH_LIMIT = -0; // TODO make better
        public static final double EXTEND_LOW_LIMIT = 0;
        public static final double ANGLE_HIGH_LIMIT = -0; // TODO make better
        public static final double ANGLE_LOW_LIMIT = 0;
        public static final double ANGLE_HOOK_LENGTH = 35 * METERS_PER_INCH; // in meters // TODO get better measurements if possible
        public static final double DISTANCE_BETWEEN_BARS = 28.5024669985 * METERS_PER_INCH; // in meters
    }
}
