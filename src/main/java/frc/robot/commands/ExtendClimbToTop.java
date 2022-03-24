// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static frc.robot.Constants.Climber.*;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendClimbToTop extends SequentialCommandGroup {
  /** Creates a new ExtendClimbToTop. */
  public ExtendClimbToTop(Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      race(
        new RunCommand(
          () -> {
            climber.extendLeftHook(EXTEND_TO_TOP_SPEED);
            climber.extendRightHook(EXTEND_TO_TOP_SPEED);
          }, 
          climber
        ),
        new WaitUntilCommand(() -> climber.getLeftTopMagLimit() && climber.getRightTopMagLimit())
      ),
      new InstantCommand(
        () -> {
          climber.extendLeftHook(0.0);
          climber.extendRightHook(0.0);
        },
        climber
      )
    );
  }
}
