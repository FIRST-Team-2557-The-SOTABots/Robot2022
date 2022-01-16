// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Class representing the Logitech F310 controller used by our team.
 * Port numbers are provided, as well as useful method changes 
 */
public class Logitech extends Joystick {

    public static class Ports {
        // trigger ports, output values range from 0 to 1
        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;

        // button ports
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;

        // bumper ports
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;

        // back and start ports
        public static final int BACK = 7;
        public static final int START = 8;

        // axis numbers and button port for left stick
        public static final int LEFT_STICK_X = 0;
        public static final int LEFT_STICK_Y = 1;
        public static final int LEFT_STICK_BUTTON = 9;

        // axis numbers and button port for right stick
        public static final int RIGHT_STICK_X = 4;
        public static final int RIGHT_STICK_Y = 5;
        public static final int RIGHT_STICK_BUTTON = 10;
    }
    

    // map recording deadband values for each axis
    private HashMap<Integer, Double> deadbandMap = new HashMap<>();

    public Logitech(int port) {
        super(port);
    }

    /**
     * modifies output of getRawAxis so that upwards on the joysticks outputs 
     * positive values in alignment with convention
     * @param axis the axis number
     * @return the value of the axis
     */
    @Override
    public double getRawAxis(int axis) {
        double value;
        if (axis == Ports.LEFT_STICK_Y || axis == Ports.RIGHT_STICK_Y)
            value = -super.getRawAxis(axis);
        else value = super.getRawAxis(axis);

        if (deadbandMap.containsKey(axis))
            return Math.abs(value) > deadbandMap.get(axis) ? value : 0;
        else return value;            
    }

    /**
     * sets the deadband value for an axis
     * @param axis the axis to set the deadband for, must be between 0 and 5 inclusive
     * @param deadband the deadband value to set, must be between 0 and 1.0 inclusive
     */
    public void setDeadband(int axis, double deadband) {
        if (axis < 0 || axis > 5 || deadband < 0 || deadband > 1.0) throw new IllegalArgumentException();
        deadbandMap.put(axis, deadband);
    }
}
