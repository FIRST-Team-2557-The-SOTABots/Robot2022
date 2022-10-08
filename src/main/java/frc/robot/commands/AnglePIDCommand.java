// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Climber.AngleMovement;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AnglePIDCommand extends ParallelRaceGroup {
  /** Creates a new AnglePIDCommand. */
  public AnglePIDCommand(Climber climber, AngleMovement angleMovement) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> SmartDashboard.putNumber("setpoint", angleMovement.setpoint)),
      new PIDCommand(
        new PIDController(angleMovement.kp, angleMovement.ki, angleMovement.kd),
        () -> climber.getAngleEncoderPosition(), 
        angleMovement.setpoint,
        (double output) -> {
          climber.runAngle(output);
        }
      ),
      new WaitUntilCommand(
        () -> {
          if (angleMovement == AngleMovement.HOLD_HIGH)
            return false;
          else
            return Math.abs(angleMovement.setpoint - climber.getAngleEncoderPosition()) < angleMovement.tolerance;
        }
      )
    );
  }
}
