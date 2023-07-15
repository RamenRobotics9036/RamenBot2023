package frc.robot.simulation.motor;

import frc.robot.simulation.framework.SimManagerBase;

/**
 * Simulation manager for a simple motor, AND an encoder that reads that motor position.
 */
public class MotorSimManager extends SimManagerBase<Double, Double> {
  private final MotorSimModel m_model;

  /**
   * Constructor.
   */
  public MotorSimManager() {
    m_model = new MotorSimModel();
  }

  @Override
  protected Double doSimulation(Double motorPowerPercentage) {
    // No need to call super, since it's abstract class and doesn't
    // implement doSimulation()

    double newEncoderPosition = m_model.updateMotorPosition(motorPowerPercentage);

    return newEncoderPosition;
  }
}
