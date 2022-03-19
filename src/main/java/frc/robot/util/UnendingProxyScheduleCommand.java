// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Schedules the given command repeatedly, with no end condition. The command is scheduled as interruptible.
 */
public class UnendingProxyScheduleCommand extends CommandBase {
  private final Command m_toSchedule;

  /**
   * Creates a new UnendingProxyScheduleCommand that schedules the given command repeatedly, and never ends.
   * Interrupting this command with a command group or decorator will end it and cancel the given command.
   *
   * @param toSchedule the commands to schedule
   */
  public UnendingProxyScheduleCommand(Command toSchedule) {
    m_toSchedule = toSchedule;
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_toSchedule.cancel();
    }
  }

  @Override
  public void execute() {
    if (!m_toSchedule.isScheduled()) {
      m_toSchedule.schedule();
      SmartDashboard.putNumber("upsc scheduled", Timer.getFPGATimestamp());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
