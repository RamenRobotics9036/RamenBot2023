package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import frc.robot.Constants;
import frc.robot.commands.CloseGrabberCommand;
import frc.robot.commands.GrabberOpenCommand;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Subclass of GrabberSystem that is used for simulation. Note that this code isn't run if
 * the robot is not running in simulation mode.
 */
public class GrabberSystemSim extends GrabberSystem {
  private Value m_solenoidStatus;
  private boolean m_grabberPhysicallyOpened;
  private REVPHSim m_penumaticSim;
  private DoubleSolenoidSim m_solenoidSim;

  /**
   * Factory method to create a GrabberSystemSim or GrabberSystem object.
   */
  public static GrabberSystem createGrabberSystemInstance(int grabberForwardChannel,
      int grabberBackwardChannel,
      XboxController controller) {
    GrabberSystem result;

    if (RobotBase.isSimulation()) {
      result = new GrabberSystemSim(grabberForwardChannel, grabberBackwardChannel, controller);

      System.out.println("GRABBERSYSTEM: **** Simulation ****");

    }
    else {
      result = new GrabberSystem(grabberForwardChannel, grabberBackwardChannel, controller);

      System.out.println("GRABBERSYSTEM: Physical Robot version");
    }

    return result;
  }

  /**
   * Constructor.
   */
  public GrabberSystemSim(int grabberForwardChannel,
      int grabberBackwardChannel,
      XboxController controller) {
    // FIRST, we call superclass
    super(grabberForwardChannel, grabberBackwardChannel, controller);

    // This entire class should only be instantiated when we're under simulation.
    // But just in-case someone tries to instantiate it otherwise, we do an extra
    // check here.
    if (!RobotBase.isSimulation()) {
      return;
    }

    // Create simulated pneumatic hub
    m_penumaticSim = new REVPHSim(m_pneumaticHub);

    // Create simulated solenoid
    m_solenoidSim = new DoubleSolenoidSim(m_penumaticSim, grabberForwardChannel,
        grabberBackwardChannel);

    m_solenoidStatus = Value.kOff;
    m_grabberPhysicallyOpened = Constants.SimConstants.kgrabberInitiallyOpened;
  }

  private void addCommandButtons() {
    // Open grabber
    Shuffleboard.getTab("Simulation")
        .add(Constants.SimWidgets.kOpenGrabber.m_name, new GrabberOpenCommand(this))
        .withWidget(BuiltInWidgets.kCommand)
        .withPosition(Constants.SimWidgets.kOpenGrabber.m_xpos,
            Constants.SimWidgets.kOpenGrabber.m_ypos)
        .withSize(Constants.SimWidgets.kOpenGrabber.m_width,
            Constants.SimWidgets.kOpenGrabber.m_height);

    // Close grabber
    Shuffleboard.getTab("Simulation")
        .add(Constants.SimWidgets.kCloseGrabber.m_name, new CloseGrabberCommand(this))
        .withWidget(BuiltInWidgets.kCommand)
        .withPosition(Constants.SimWidgets.kCloseGrabber.m_xpos,
            Constants.SimWidgets.kCloseGrabber.m_ypos)
        .withSize(Constants.SimWidgets.kCloseGrabber.m_width,
            Constants.SimWidgets.kCloseGrabber.m_height);

    // Grabber commands
    Shuffleboard.getTab("Simulation").add(Constants.SimWidgets.kGrabberSystemCommands.m_name, this)
        .withPosition(Constants.SimWidgets.kGrabberSystemCommands.m_xpos,
            Constants.SimWidgets.kGrabberSystemCommands.m_ypos)
        .withSize(Constants.SimWidgets.kGrabberSystemCommands.m_width,
            Constants.SimWidgets.kGrabberSystemCommands.m_height);
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

  private void addShuffleboardWidgets() {
    // Grabber functional
    Shuffleboard.getTab("Simulation")
        .addBoolean(Constants.SimWidgets.kGrabberFunctional.m_name, () -> true)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#8B0000"))
        .withPosition(Constants.SimWidgets.kGrabberFunctional.m_xpos,
            Constants.SimWidgets.kGrabberFunctional.m_ypos)
        .withSize(Constants.SimWidgets.kGrabberFunctional.m_width,
            Constants.SimWidgets.kGrabberFunctional.m_height);

    // Grabber open/closed
    Shuffleboard.getTab("Simulation")
        .addString(Constants.SimWidgets.kGrabber.m_name, () -> getGrabberStatusText())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(Constants.SimWidgets.kGrabber.m_xpos, Constants.SimWidgets.kGrabber.m_ypos)
        .withSize(Constants.SimWidgets.kGrabber.m_width, Constants.SimWidgets.kGrabber.m_height);
  }

  private boolean isRobotEnabled() {
    return RobotState.isEnabled();
  }

  @Override
  public void initDashBoard() {
    super.initDashBoard();

    addShuffleboardWidgets();
    addCommandButtons();
  }

  @Override
  public void periodic() {
    super.periodic();

    // When Robot is disabled, the entire simulation freezes
    if (isRobotEnabled()) {
      // Do nothing
    }
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // When Robot is disabled, the entire simulation freezes
    if (isRobotEnabled()) {
      m_solenoidStatus = m_solenoidSim.get();

      // If the solenoid is on, update the physicalGrabber as opened or closed
      if (m_solenoidStatus == Value.kForward) {
        m_grabberPhysicallyOpened = true;
      }
      else if (m_solenoidStatus == Value.kReverse) {
        m_grabberPhysicallyOpened = false;
      }
    }
  }
}
