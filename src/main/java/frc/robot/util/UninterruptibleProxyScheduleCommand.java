// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Schedules the given commands when this command is initialized, and ends when all the commands are
 * no longer scheduled. Useful for forking off from CommandGroups. If this command is interrupted,
 * it will cancel all of the commands.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class UninterruptibleProxyScheduleCommand extends CommandBase {
  private final Set<Command> m_toSchedule;
  private boolean m_finished;

  /**
   * Creates a new ProxyScheduleCommand that schedules the given commands when initialized, and ends
   * when they are all no longer scheduled.
   *
   * @param toSchedule the commands to schedule
   */
  public UninterruptibleProxyScheduleCommand(Command... toSchedule) {
    m_toSchedule = Set.of(toSchedule);
  }

  @Override
  public void initialize() {
    for (Command command : m_toSchedule) {
      command.schedule(false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      for (Command command : m_toSchedule) {
        command.cancel();
      }
    }
  }

  @Override
  public void execute() {
    m_finished = true;
    for (Command command : m_toSchedule) {
      m_finished &= !command.isScheduled();
    }
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
