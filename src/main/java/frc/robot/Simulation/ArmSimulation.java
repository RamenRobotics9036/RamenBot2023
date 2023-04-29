package frc.robot.Simulation;

public class ArmSimulation {
  private WinchSimulation m_winchSimulation;
  private double m_topRotationsLimit;
  private double m_bottomRotationsLimit;
  private double m_deltaRotationsBeforeBroken;

  // Constructor
  public ArmSimulation(
    WinchSimulation winchSimulation,
    double topRotationsLimit,
    double bottomRotationsLimit,
    double deltaRotationsBeforeBroken) {

    if (winchSimulation == null) {
      throw new IllegalArgumentException("winchSimulation");
    }

    if (bottomRotationsLimit > 1 || bottomRotationsLimit < 0 || topRotationsLimit > 1 || topRotationsLimit < 0) {
      throw new IllegalArgumentException("bottomRotationsLimit and topRotationsLimit must be <=1");
    }

    if (deltaRotationsBeforeBroken > 1 || deltaRotationsBeforeBroken < 0) {
      throw new IllegalArgumentException("deltaRotationsBeforeBroken must be <=1");
    }

    if (topRotationsLimit <= bottomRotationsLimit) {
      throw new IllegalArgumentException("topRotationsLimit must be greater than bottomRotationsLimit");
    }

    if (topRotationsLimit + deltaRotationsBeforeBroken > 1) {
      throw new IllegalArgumentException("topRotationsLimit + deltaRotationsBeforeBroken must be <=1");
    }

    if (bottomRotationsLimit - deltaRotationsBeforeBroken < 0) {
      throw new IllegalArgumentException("bottomRotationsLimit - deltaRotationsBeforeBroken must be >= 0");
    }

    m_winchSimulation = winchSimulation;
    m_topRotationsLimit = topRotationsLimit;
    m_bottomRotationsLimit = bottomRotationsLimit;
    m_deltaRotationsBeforeBroken = deltaRotationsBeforeBroken;
  }
}
