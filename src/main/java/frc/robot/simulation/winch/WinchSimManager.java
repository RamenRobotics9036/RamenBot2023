package frc.robot.simulation.winch;

import frc.robot.simulation.framework.SimManagerBase;
import frc.robot.simulation.winch.WinchSimModel.WindingOrientation;
import frc.robot.subsystems.RelativeEncoderSim;

/**
 * Simulates a winch. The winch depends on a motor (not part of this particular simulation),
 * and outputs the length of the string.
 */
public class WinchSimManager extends SimManagerBase<Double, WinchState> {
  private final WinchSimModel m_model;
  private WinchState m_winchStateCopy;

  /**
   * Constructor.
   */
  public WinchSimManager(RelativeEncoderSim motorEncoderSim, // $TODO No need to pass in
      double spoolDiameterMeters,
      double totalStringLenMeters,
      double initialLenSpooled,
      WindingOrientation initialWindingOrientation,
      boolean invertMotor) {

    m_winchStateCopy = new WinchState(totalStringLenMeters);

    m_model = new WinchSimModel(motorEncoderSim, spoolDiameterMeters, totalStringLenMeters,
        initialLenSpooled, initialWindingOrientation, invertMotor);
  }

  @Override
  protected WinchState doSimulation(Double winchMotorEncoderRotations) {
    // No need to call super, since it's abstract class and doesn't
    // implement doSimulation()

    // $TODO - Needto pass the input in!
    m_model.updateNewLenSpooled();

    // Update our WinchState copy to avoid creating a new object
    m_winchStateCopy.setStringUnspooledLen(m_model.getStringUnspooledLen());
    m_winchStateCopy.setWindingOrientation(m_model.getWindingOrientation());
    m_winchStateCopy.setIsBroken(m_model.getIsBroken());

    return m_winchStateCopy;
  }
}
