package frc.robot.simulation.motor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Does the real-world simulation for the motor.
 */
public class MotorSimModel {
  private final DCMotor m_extenderMotorModel;
  private final DCMotorSim m_extenderMotorSim;

  /**
   * Constructor.
   */
  public MotorSimModel() {
    // Model a NEO motor (or any other motor)
    m_extenderMotorModel = DCMotor.getNEO(1); // 1 motor in the gearbox

    // Create the motor simulation with motor model, gear ratio, and moment of
    // inertia
    double motorMomentInertia = 0.0005;
    m_extenderMotorSim = new DCMotorSim(m_extenderMotorModel,
        Constants.SimConstants.kextenderSimGearRatio, motorMomentInertia);
  }

  /**
   * Runs 20ms simulation of the motor, and then returns the new encoder position (in Rotations).
   */
  public double updateMotorPosition(double motorPowerPercentage) {
    // Calculate the input voltage for the motor
    double inputVoltageVolts = motorPowerPercentage * 12.0;

    // Update the motor simulation
    m_extenderMotorSim.setInput(inputVoltageVolts);
    m_extenderMotorSim.update(0.02);

    // Update the Encoder based on the simulation - the units are "number of
    // rotations"
    return m_extenderMotorSim.getAngularPositionRotations();
  }
}
