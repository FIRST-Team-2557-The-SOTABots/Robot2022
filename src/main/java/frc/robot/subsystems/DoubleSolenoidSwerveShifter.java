package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

public class DoubleSolenoidSwerveShifter implements SwerveShifter{

    private final DoubleSolenoid shifter;

    public DoubleSolenoidSwerveShifter(DoubleSolenoid shifter){
        this.shifter = shifter;
    }

    

    public int getGear(){
        return this.shifter.get() == Constants.Swerve.HIGH_GEAR_VALUE ? 1 : 0;
    }

    public void shift(int value){
        value = MathUtil.clamp(value, 0, 1);
        shifter.set(value == 0 ? Constants.Swerve.LOW_GEAR_VALUE : Constants.Swerve.HIGH_GEAR_VALUE);
    }
}
