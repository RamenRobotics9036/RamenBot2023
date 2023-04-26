package frc.robot.Simulation;

public class ArmSimulation {
  private WinchSimulation m_winchSimulation;
  private double m_topAngleDegrees;
  private double m_bottomAngleDegrees;
  private double m_deltaDegreesBeforeBroken;

  // Constructor
  public ArmSimulation(
    WinchSimulation winchSimulation,
    double topAngleDegrees,
    double bottomAngleDegrees,
    double deltaDegreesBeforeBroken) {

    if (winchSimulation == null) {
      throw new IllegalArgumentException("winchSimulation");
    }

    if (bottomAngleDegrees > 360 || bottomAngleDegrees < 0 || topAngleDegrees > 360 || topAngleDegrees < 0) {
      throw new IllegalArgumentException("bottomAngleDegrees and topAngleDegrees must be <=360");
    }

    if (deltaDegreesBeforeBroken > 360 || deltaDegreesBeforeBroken < 0) {
      throw new IllegalArgumentException("deltaDegreesBeforeBroken must be <=360");
    }

    if (topAngleDegrees <= bottomAngleDegrees) {
      throw new IllegalArgumentException("topAngleDegrees must be greater than bottomAngleDegrees");
    }

    if (topAngleDegrees + deltaDegreesBeforeBroken > 360) {
      throw new IllegalArgumentException("topAngleDegrees + deltaDegreesBeforeBroken must be <=360");
    }

    if (bottomAngleDegrees - deltaDegreesBeforeBroken < 0) {
      throw new IllegalArgumentException("bottomAngleDegrees - deltaDegreesBeforeBroken must be >= 0");
    }

    m_winchSimulation = winchSimulation;
    m_topAngleDegrees = topAngleDegrees;
    m_bottomAngleDegrees = bottomAngleDegrees;
    m_deltaDegreesBeforeBroken = deltaDegreesBeforeBroken;
  }
}
