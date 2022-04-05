// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.Constants.Auto.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallSteal extends SequentialCommandGroup {
  /** Creates a new TwoBallSteal. */
  public TwoBallSteal(SwerveDrive swerveDrive, Delivery delivery, Intake intake, Shooter shooter, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
        () -> {
          shooter.hoodUp();
          swerveDrive.shiftUp();
          swerveDrive.setPose(path2A.getInitialState());
        }, 
        swerveDrive
      ),
      deadline(
        new SwerveControllerCommand(swerveDrive, path2A),
        new RunIntakeCommand(intake),
        new RevFlywheelCommand(shooter),
        new IndexCommand(delivery, intake)
      ),
      new EndSwerveCommand(swerveDrive),
      new EndIntakeCommand(intake),
      new AutoShootCommand(shooter, swerveDrive, delivery, limelight).withTimeout(PATH_2_SHOOT_1_DURATION),
      new EndShooterCommand(shooter),
      new EndDeliveryCommand(delivery),
      deadline(
        new SwerveControllerCommand(swerveDrive, path2B),
        new RunIntakeCommand(intake),
        new IndexCommand(delivery, intake)
      ),
      new EndSwerveCommand(swerveDrive),
      new RunOuttakeCommand(intake, delivery).withTimeout(PATH_2_OUTTAKE_2_DURATION),
      new InstantCommand(
        () -> {
          intake.retract();
          delivery.runMotor(0.0);
        },
        intake
      ).asProxy(), // Maybe make a seperate stop delivery command, but cant see a problem with this
      new SwerveControllerCommand(swerveDrive, Path2C),
      new EndSwerveCommand(swerveDrive)
    );
  }
}
