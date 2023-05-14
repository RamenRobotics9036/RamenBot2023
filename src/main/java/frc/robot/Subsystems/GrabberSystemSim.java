package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;
import java.util.function.BooleanSupplier;

import frc.robot.Constants;

public class GrabberSystemSim extends GrabberSystem {
  private Value m_solenoidStatus;
  private boolean m_grabberPhysicallyOpened;

  public static GrabberSystem CreateGrabberSystemInstance(int grabberForwardChannel, int grabberBackwardChannel,
      XboxController controller) {
    GrabberSystem result;

    if (RobotBase.isSimulation()) {
      result = new GrabberSystemSim(
          grabberForwardChannel,
          grabberBackwardChannel,
          controller);

      System.out.println("GRABBERSYSTEM: **** Simulation ****");

    } else {
      result = new GrabberSystem(
          grabberForwardChannel,
          grabberBackwardChannel,
          controller);

      System.out.println("GRABBERSYSTEM: Physical Robot version");
    }

    return result;
  }

  // Constructor
  public GrabberSystemSim(int grabberForwardChannel, int grabberBackwardChannel, XboxController controller) {
    // FIRST, we call superclass
    super(grabberForwardChannel,
        grabberBackwardChannel,
        controller);

    // This entire class should only be instantiated when we're under simulation.
    // But just in-case someone tries to instantiate it otherwise, we do an extra
    // check here.
    if (!RobotBase.isSimulation()) {
      return;
    }

    m_solenoidStatus = Value.kOff;
    m_grabberPhysicallyOpened = Constants.SimConstants.kgrabberInitiallyOpened;
  }

  public BooleanSupplier getGrabberOpenSupplier() {
    return () -> m_grabberPhysicallyOpened;
  }

  private String getGrabberStatusText() {
    String solenoidStatusText;
    String physicalGrabberText;

    switch (m_solenoidStatus) {
      case kForward:
        solenoidStatusText = "Forward";
        break;
      case kReverse:
        solenoidStatusText = "Reverse";
        break;
      case kOff:
        solenoidStatusText = "Off";
        break;
      default:
        solenoidStatusText = "Unknown";
        break;
    }

    physicalGrabberText = m_grabberPhysicallyOpened ? "Open" : "Closed";

    return String.format("%s (Sol: %s)", physicalGrabberText, solenoidStatusText);
  }

  private void AddShuffleboardWidgets() {
    Shuffleboard.getTab("Simulation")
        .addString(Constants.SimWidgets.kGrabber.name, () -> getGrabberStatusText())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(Constants.SimWidgets.kGrabber.x, Constants.SimWidgets.kGrabber.y)
        .withSize(Constants.SimWidgets.kGrabber.width, Constants.SimWidgets.kGrabber.height);
  }

  private boolean isRobotEnabled() {
    return RobotState.isEnabled();
  }

  @Override
  public void initDashBoard() {
    super.initDashBoard();

    AddShuffleboardWidgets();
  }

  @Override
  public void periodic() {
    super.periodic();

    // When Robot is disabled, the entire simulation freezes
    if (isRobotEnabled()) {
    }
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // When Robot is disabled, the entire simulation freezes
    if (isRobotEnabled()) {
      m_solenoidStatus = m_solenoid.get();

      // If the solenoid is on, update the physicalGrabber as opened or closed
      if (m_solenoidStatus == Value.kForward) {
        m_grabberPhysicallyOpened = true;
      } else if (m_solenoidStatus == Value.kReverse) {
        m_grabberPhysicallyOpened = false;
      }
    }
  }
}
