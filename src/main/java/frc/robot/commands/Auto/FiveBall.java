// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.Constants.Auto.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBall extends SequentialCommandGroup {



  /** Creates a new FiveBall. */
  public FiveBall(SwerveDrive swerveDrive, Delivery delivery, Intake intake, Shooter shooter, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
        () -> {
          shooter.hoodUp();
          swerveDrive.shiftUp();
          swerveDrive.setPose(path1A.getInitialState());
          swerveDrive.setFieldCentricActive(false);
        }, 
        swerveDrive
      ),
      deadline(
        new SwerveControllerCommand(swerveDrive, path1A),
        new RunIntakeCommand(intake),
        new RevFlywheelCommand(shooter),
        new IndexCommand(delivery, intake)
      ),
      new EndSwerveCommand(swerveDrive),
      new EndIntakeCommand(intake),
      new AutoShootCommand(shooter, swerveDrive, delivery, limelight).withTimeout(Constants.Auto.PATH_1_SHOOT_1_DURATION),
      new EndShooterCommand(shooter),
      new EndDeliveryCommand(delivery),
      deadline(
        new SwerveControllerCommand(swerveDrive, path1B),
        new RunIntakeCommand(intake),
        new RevFlywheelCommand(shooter),
        new IndexCommand(delivery, intake)
      ),
      new EndSwerveCommand(swerveDrive),
      new EndIntakeCommand(intake),
      new AutoShootCommand(shooter, swerveDrive, delivery, limelight).withTimeout(Constants.Auto.PATH_1_SHOOT_2_DURATION),
      new EndShooterCommand(shooter),
      new EndDeliveryCommand(delivery),
      deadline(
        new SwerveControllerCommand(swerveDrive, path1C),
        new RunIntakeCommand(intake),
        new IndexCommand(delivery, intake)
      ),
      new EndSwerveCommand(swerveDrive),
      deadline(
        new RunIntakeCommand(intake).withTimeout(Constants.Auto.HUMAN_PLAYER_WAIT_TIME),
        new IndexCommand(delivery, intake)
      ),
      deadline(
        new SwerveControllerCommand(swerveDrive, path1D),
        new RevFlywheelCommand(shooter),
        new RunCommand(
          () -> {
            intake.run(0.0);
            intake.extend();
          }
        )
      ),
      new EndSwerveCommand(swerveDrive),
      new AutoShootCommand(shooter, swerveDrive, delivery, limelight).withTimeout(Constants.Auto.PATH_1_SHOOT_3_DURATION),
      new EndShooterCommand(shooter),
      new EndDeliveryCommand(delivery)
    );

    addRequirements(swerveDrive, shooter, delivery, intake, limelight);
    
  }
}
