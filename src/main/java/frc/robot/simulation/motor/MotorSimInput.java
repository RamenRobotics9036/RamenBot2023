package frc.robot.simulation.motor;

import com.revrobotics.CANSparkMax;
import frc.robot.simulation.framework.SimInputInterface;

/**
 * Helper class to implement SimInputDoubleInterface.
 */
public class MotorSimInput implements SimInputInterface<Double> {
  private CANSparkMax m_motorReal;

  /**
   * Constructor.
   */
  public MotorSimInput(CANSparkMax motorReal) {
    if (motorReal == null) {
      throw new IllegalArgumentException("motorReal cannot be null");
    }

    m_motorReal = motorReal;
  }

  @Override
  public Double getInput() {
    return m_motorReal.get();
  }
}
