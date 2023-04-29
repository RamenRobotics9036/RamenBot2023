package frc.robot.Simulation;

public class ArmSimulation {
  private WinchSimulation m_winchSimulation;
  private double m_topAngleRotations;
  private double m_bottomAngleRotations;
  private double m_deltaRotationsBeforeBroken;

  // Constructor
  public ArmSimulation(
    WinchSimulation winchSimulation,
    double topAngleRotations,
    double bottomAngleRotations,
    double deltaRotationsBeforeBroken) {

    if (winchSimulation == null) {
      throw new IllegalArgumentException("winchSimulation");
    }

    if (bottomAngleRotations > 1 || bottomAngleRotations < 0 || topAngleRotations > 1 || topAngleRotations < 0) {
      throw new IllegalArgumentException("bottomAngleRotations and topAngleRotations must be <=1");
    }

    if (deltaRotationsBeforeBroken > 1 || deltaRotationsBeforeBroken < 0) {
      throw new IllegalArgumentException("deltaRotationsBeforeBroken must be <=1");
    }

    if (topAngleRotations <= bottomAngleRotations) {
      throw new IllegalArgumentException("topAngleRotations must be greater than bottomAngleRotations");
    }

    if (topAngleRotations + deltaRotationsBeforeBroken > 1) {
      throw new IllegalArgumentException("topAngleRotations + deltaRotationsBeforeBroken must be <=1");
    }

    if (bottomAngleRotations - deltaRotationsBeforeBroken < 0) {
      throw new IllegalArgumentException("bottomAngleRotations - deltaRotationsBeforeBroken must be >= 0");
    }

    m_winchSimulation = winchSimulation;
    m_topAngleRotations = topAngleRotations;
    m_bottomAngleRotations = bottomAngleRotations;
    m_deltaRotationsBeforeBroken = deltaRotationsBeforeBroken;
  }
}
