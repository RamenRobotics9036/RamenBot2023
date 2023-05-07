package frc.robot.Simulation;

import frc.robot.Subsystems.RelativeEncoderSim;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;

// $TODO - Remember not to run the simulation during Robot disabled state
// $TODO - Remember that we update the ShuffleBoard in RobotPeriodic, not SimulationPeriodic
//
// Basic idea of the winch string:
// |-----------------( )-----------------|
//                    ^ (Center is fully unwound string)
//   ^ (Mostly spooled on back)
//        Back <--         --> Front
//
public class WinchSimulation {
  public enum StringOrientation {
    BackOfRobot,
    FrontOfRobot
  }

  private RelativeEncoderSim m_motorEncoderSim;
  private double m_SpoolDiameterMeters;
  private double m_TotalStringLenMeters;
  private double m_CurrentLenSpooled;
  private boolean m_IsBroken;
  private double m_initialMotorRotations;
  private double m_initialLenSpooled;
  private double m_motorPolarity;

  // Constructor
  public WinchSimulation(
    RelativeEncoderSim motorEncoderSim,
    double SpoolDiameterMeters,
    double TotalStringLenMeters,
    double InitialLenSpooled,
    StringOrientation InitialStringOrientation,
    boolean invertMotor) {

    // Sanity checks
    if (motorEncoderSim == null) {
      throw new IllegalArgumentException("motorEncoderSim is null");
    }

    if (SpoolDiameterMeters <= 0) {
      throw new IllegalArgumentException("SpoolDiameterMeters must be >0");
    }

    if (TotalStringLenMeters <= 0) {
      throw new IllegalArgumentException("TotalStringLenMeters must be >0");
    }

    if (InitialLenSpooled < 0) {
      throw new IllegalArgumentException("InitialLenSpooled must be >=0");
    }

    if (InitialLenSpooled > TotalStringLenMeters) {
      throw new IllegalArgumentException("InitialLenSpooled must be <= TotalStringLenMeters");
    }

    m_motorEncoderSim = motorEncoderSim;
    m_SpoolDiameterMeters = SpoolDiameterMeters;
    m_TotalStringLenMeters = TotalStringLenMeters;
    m_motorPolarity = invertMotor ? -1 : 1;

    // If the string is towards the back of the robot, then we represent the length of string pooled as a NEGATIVE number
    // See diagram above
    m_initialLenSpooled = (InitialStringOrientation == StringOrientation.BackOfRobot) ?
      -1 * InitialLenSpooled :
      InitialLenSpooled;

    m_IsBroken = false;

    // Take a snapshot of current DCMotor position
    m_initialMotorRotations = m_motorEncoderSim.getPosition();

    // Call this to initialize m_currentLenSpooled
    UpdateNewLenSpooled();
  }

  public double GetStringExtendedLen() {
    return m_TotalStringLenMeters - Math.abs(m_CurrentLenSpooled);
  }

  public double GetStringExtendedPercent() {
    return GetStringExtendedLen() / m_TotalStringLenMeters;
  }

  public StringOrientation GetStringOrientation() {
    // We define 0 as string orientation: back
    return (m_CurrentLenSpooled <= 0) ?
      StringOrientation.BackOfRobot :
      StringOrientation.FrontOfRobot;
  }

  public String GetStringOrientationName() {
    return GetStringOrientation().name();
  }

  public boolean GetIsBroken() {
    return m_IsBroken;
  }
  
  public void periodic() {
  }

  private void UpdateNewLenSpooled() {
    double currentRotations;
    double deltaRotations;
    double deltaStringLenMeters;
    double newCurrentLenSpooled;

    // If the winch is broken, there's nothing to update
    if (m_IsBroken) {
      return;
    }

    // How much has the motor turned since winch initialized?
    currentRotations = m_motorEncoderSim.getPosition() * m_motorPolarity;
    deltaRotations = currentRotations - m_initialMotorRotations;

    // How much sting-length (in meters) has been spooled or unspooled?
    deltaStringLenMeters = deltaRotations * (Math.PI * m_SpoolDiameterMeters);
    newCurrentLenSpooled = m_initialLenSpooled + deltaStringLenMeters;

    // Check for bounds
    if (newCurrentLenSpooled > m_TotalStringLenMeters) {
      newCurrentLenSpooled = m_TotalStringLenMeters;
      m_IsBroken = true;
    }
    else if (newCurrentLenSpooled < -1 * m_TotalStringLenMeters) {
      newCurrentLenSpooled = -1 * m_TotalStringLenMeters;
      m_IsBroken = true;
    }

    m_CurrentLenSpooled = newCurrentLenSpooled;
  }
  
  public void simulationPeriodic() {
    UpdateNewLenSpooled();
  }
}
