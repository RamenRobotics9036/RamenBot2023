package frc.robot.Simulation;

import frc.robot.Subsystems.RelativeEncoderSim;

public class ExtenderSimulation {
  private RelativeEncoderSim m_motorEncoderSim;
  private double m_TotalExtenderLengthMeters = 0.5;
  private double m_MinExtendLength = 0;
  private double m_cylinderDiameterMeters;
  private double m_currentExtendedLen;
  private boolean m_IsBroken;
  private double m_initialMotorRotations;
  private double m_initialExtendedLen;
  private double m_motorPolarity;

  // Constructor
  public ExtenderSimulation(
      RelativeEncoderSim motorEncoderSim,
      double CylinderDiameterMeters,
      double TotalExtenderLengthMeters,
      double InitialExtendedLen,
      boolean invertMotor) {

    // Sanity checks
    if (motorEncoderSim == null) {
      throw new IllegalArgumentException("motorEncoderSim is null");
    }

    if (CylinderDiameterMeters <= 0) {
      throw new IllegalArgumentException("CylinderDiameterMeters must be >0");
    }

    if (TotalExtenderLengthMeters <= 0) {
      throw new IllegalArgumentException("TotalExtenderLengthMeters must be >0");
    }

    if (InitialExtendedLen < 0) {
      throw new IllegalArgumentException("InitialExtendedLen must be >=0");
    }

    if (InitialExtendedLen > TotalExtenderLengthMeters) {
      throw new IllegalArgumentException("InitialExtendedLen must be <= TotalExtenderLengthMeters");
    }

    m_motorEncoderSim = motorEncoderSim;
    m_cylinderDiameterMeters = CylinderDiameterMeters;
    m_TotalExtenderLengthMeters = TotalExtenderLengthMeters;
    m_initialExtendedLen = InitialExtendedLen;
    m_motorPolarity = invertMotor ? -1 : 1;

    m_IsBroken = false;

    // Take a snapshot of current DCMotor position
    m_initialMotorRotations = m_motorEncoderSim.getPosition();

    // Call this to initialize m_currentExtendedLen
    UpdateNewExtendedLen();
  }

  public double GetExtendedLen() {
    return m_currentExtendedLen;
  }

  public double GetExtendedPercent() {
    return GetExtendedLen() / m_TotalExtenderLengthMeters;
  }

  public boolean GetIsBroken() {
    return m_IsBroken;
  }

  public void periodic() {
  }

  private void UpdateNewExtendedLen() {
    double currentRotations;
    double deltaRotations;
    double deltaLenMeters;
    double newCurrentLen;

    // If the extender is broken, there's nothing to update
    if (m_IsBroken) {
      return;
    }

    // How much has the motor turned since extender initialized?
    currentRotations = m_motorEncoderSim.getPosition() * m_motorPolarity;
    deltaRotations = currentRotations - m_initialMotorRotations;

    deltaLenMeters = deltaRotations * (Math.PI * m_cylinderDiameterMeters);
    newCurrentLen = m_initialExtendedLen + deltaLenMeters;

    // Check for bounds
    if (newCurrentLen > m_TotalExtenderLengthMeters) {
      newCurrentLen = m_TotalExtenderLengthMeters;
      m_IsBroken = true;
    } else if (newCurrentLen < m_MinExtendLength) {
      newCurrentLen = m_MinExtendLength;
      m_IsBroken = true;
    }

    m_currentExtendedLen = newCurrentLen;
  }

  public void simulationPeriodic() {
    UpdateNewExtendedLen();
  }
}
