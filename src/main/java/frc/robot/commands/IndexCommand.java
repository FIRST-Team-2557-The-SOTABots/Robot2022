// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.util.UninterruptibleProxyScheduleCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexCommand extends SequentialCommandGroup {
  /** Creates a new DeliveryCommand. */
  public IndexCommand(Delivery delivery, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> SmartDashboard.putString("delivery", "waiting")),
      new WaitUntilCommand(() -> delivery.getSensor1() && !intake.isRetracted()), //waits till the sensor & fully extended
      parallel(
        new DeliveryCommand(delivery).withTimeout(Constants.Delivery.MAX_DELIVERY_DURATION),//runs delivery & retracts intakes
        new UninterruptibleProxyScheduleCommand(
          new RunCommand(
            () -> {
              intake.retract();
              intake.run(0.0);
            }, 
            intake
          ).withTimeout(Constants.Delivery.RETRACTED_DURATION)
          .andThen(() -> SmartDashboard.putString("delivery", "retracted"))
        )
      ),
      new WaitCommand(Constants.Delivery.COOLDOWN)
    );
  }
}
