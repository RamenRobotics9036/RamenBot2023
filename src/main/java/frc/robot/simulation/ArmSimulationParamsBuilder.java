package frc.robot.simulation;

/**
 * Builder class pattern for ArmSimulationParams.
 */
public class ArmSimulationParamsBuilder {
  @SuppressWarnings("MemberNameCheck")
  private ArmSimulationParams instance;

  /**
   * Constructor.
   */
  public ArmSimulationParamsBuilder() {
    instance = new ArmSimulationParams();
  }

  /**
   * Constructor that takes defaults as an input parameter.
   */
  public ArmSimulationParamsBuilder(ArmSimulationParams defaults) {
    instance = new ArmSimulationParams(defaults);
  }

  public ArmSimulationParamsBuilder setTopRotationsLimit(double value) {
    instance.m_topRotationsLimit = value;
    return this;
  }

  public ArmSimulationParamsBuilder setBottomRotationsLimit(double value) {
    instance.m_bottomRotationsLimit = value;
    return this;
  }

  public ArmSimulationParamsBuilder setDeltaRotationsBeforeBroken(double value) {
    instance.m_deltaRotationsBeforeBroken = value;
    return this;
  }

  public ArmSimulationParamsBuilder setGrabberBreaksIfOpenBelowThisLimit(double value) {
    instance.m_grabberBreaksIfOpenBelowThisLimit = value;
    return this;
  }

  public ArmSimulationParamsBuilder setHeightFromWinchToPivotPoint(double value) {
    instance.m_heightFromWinchToPivotPoint = value;
    return this;
  }

  public ArmSimulationParamsBuilder setArmLengthFromEdgeToPivot(double value) {
    instance.m_armLengthFromEdgeToPivot = value;
    return this;
  }

  public ArmSimulationParamsBuilder setArmLengthFromEdgeToPivotMin(double value) {
    instance.m_armLengthFromEdgeToPivotMin = value;
    return this;
  }

  public ArmSimulationParamsBuilder setEncoderRotationsOffset(double value) {
    instance.m_encoderRotationsOffset = value;
    return this;
  }

  // Return the built instance
  public ArmSimulationParams build() {
    return instance;
  }
}
