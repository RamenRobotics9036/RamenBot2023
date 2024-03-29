package frc.robot.simulation.winch;

import frc.robot.simulation.framework.SimOutputInterface;

/**
 * Winch output is the length of the string (and other proprties described
 * in WinchState class).
 */
public class WinchSimOutput implements SimOutputInterface<WinchState> {
  private final WinchState m_targWinchState;

  /**
   * Constructor.
   */
  public WinchSimOutput(WinchState targetWinchState) {
    if (targetWinchState == null) {
      throw new IllegalArgumentException("targetWinchState cannot be null");
    }

    m_targWinchState = targetWinchState;
  }

  @Override
  public void setOutput(WinchState newWinchState) {
    // Copy from output to target.
    // WinchState has a bunch of information, such as the length of string that is unpooled out,
    // and whether the winch is broken.
    m_targWinchState.copyFrom(newWinchState);
  }
}
