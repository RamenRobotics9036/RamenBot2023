package frc.robot.simulation;

/**
 * Class that holds paramaters for the arm simulation class.
 */
public class ArmSimulationParams {
  public double m_topRotationsLimit;
  public double m_bottomRotationsLimit;
  public double m_deltaRotationsBeforeBroken;
  public double m_grabberBreaksIfOpenBelowThisLimit;
  public double m_heightFromWinchToPivotPoint;
  public double m_armLengthFromEdgeToPivot;
  public double m_armLengthFromEdgeToPivotMin;
  public double m_encoderRotationsOffset;

  /**
   * Constructor with 0 params.
   */
  public ArmSimulationParams() {
  }

  /**
   * Constructor with all params.
   */
  public ArmSimulationParams(double topRotationsLimit,
      double bottomRotationsLimit,
      double deltaRotationsBeforeBroken,
      double grabberBreaksIfOpenBelowThisLimit,
      double heightFromWinchToPivotPoint,
      double armLengthFromEdgeToPivot,
      double armLengthFromEdgeToPivotMin,
      double encoderRotationsOffset) {

    m_topRotationsLimit = topRotationsLimit;
    m_bottomRotationsLimit = bottomRotationsLimit;
    m_deltaRotationsBeforeBroken = deltaRotationsBeforeBroken;
    m_grabberBreaksIfOpenBelowThisLimit = grabberBreaksIfOpenBelowThisLimit;
    m_heightFromWinchToPivotPoint = heightFromWinchToPivotPoint;
    m_armLengthFromEdgeToPivot = armLengthFromEdgeToPivot;
    m_armLengthFromEdgeToPivotMin = armLengthFromEdgeToPivotMin;
    m_encoderRotationsOffset = encoderRotationsOffset;
  }

  /**
   * Copy constructor.
   */
  public ArmSimulationParams(ArmSimulationParams other) {
    m_topRotationsLimit = other.m_topRotationsLimit; // $TODO - This should be passed-in in degrees
    m_bottomRotationsLimit = other.m_bottomRotationsLimit;
    m_deltaRotationsBeforeBroken = other.m_deltaRotationsBeforeBroken;
    m_grabberBreaksIfOpenBelowThisLimit = other.m_grabberBreaksIfOpenBelowThisLimit;
    m_heightFromWinchToPivotPoint = other.m_heightFromWinchToPivotPoint;
    m_armLengthFromEdgeToPivot = other.m_armLengthFromEdgeToPivot;
    m_armLengthFromEdgeToPivotMin = other.m_armLengthFromEdgeToPivotMin;
    m_encoderRotationsOffset = other.m_encoderRotationsOffset;
  }
}
