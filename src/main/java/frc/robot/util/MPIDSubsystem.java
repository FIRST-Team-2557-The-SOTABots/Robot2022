// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that uses a {@link PIDController} to control an output. The controller is run
 * synchronously from the subsystem's periodic() method.
 */
public abstract class MPIDSubsystem extends SubsystemBase {
  protected final MPIDController m_controller;
  protected boolean m_enabled;

  private double m_setpoint;

  /**
   * Creates a new PIDSubsystem.
   *
   * @param controller the PIDController to use
   * @param initialPosition the initial setpoint of the subsystem
   */
  public MPIDSubsystem(MPIDController controller, double initialPosition) {
    setSetpoint(initialPosition);
    m_controller = requireNonNullParam(controller, "controller", "PIDSubsystem");
    addChild("PID Controller", m_controller);
  }

  /**
   * Creates a new PIDSubsystem. Initial setpoint is zero.
   *
   * @param controller the PIDController to use
   */
  public MPIDSubsystem(MPIDController controller) {
    this(controller, 0);
  }

  @Override
  public void periodic() {
    if (m_enabled) {
      useOutput(m_controller.calculate(getMeasurement(), m_setpoint), m_setpoint);
    }
  }

  public MPIDController getController() {
    return m_controller;
  }

  /**
   * Sets the setpoint for the subsystem.
   *
   * @param setpoint the setpoint for the subsystem
   */
  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
  }

  /**
   * Returns the current setpoint of the subsystem.
   *
   * @return The current setpoint
   */
  public double getSetpoint() {
    return m_setpoint;
  }

  /**
   * Uses the output from the PIDController.
   *
   * @param output the output of the PIDController
   * @param setpoint the setpoint of the PIDController (for feedforward)
   */
  protected abstract void useOutput(double output, double setpoint);

  /**
   * Returns the measurement of the process variable used by the PIDController.
   *
   * @return the measurement of the process variable
   */
  protected abstract double getMeasurement();

  /** Enables the PID control. Resets the controller. */
  public void enable() {
    m_enabled = true;
    m_controller.reset();
  }

  /** Disables the PID control. Sets output to zero. */
  public void disable() {
    m_enabled = false;
    useOutput(0, 0);
  }

  /**
   * Returns whether the controller is enabled.
   *
   * @return Whether the controller is enabled.
   */
  public boolean isEnabled() {
    return m_enabled;
  }
}
