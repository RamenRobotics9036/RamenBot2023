package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.commands.ArmExtendFully;
import frc.robot.commands.ArmToGround;
import frc.robot.commands.ArmToMiddleNodeCone;
import frc.robot.commands.RetractArmCommand;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This adds shuffleboard widgets to the ArmSystemSim class.
 */
public class ArmSystemSimWithWidgets extends ArmSystemSim {
  private static class SendableArmPosition implements Sendable {
    private BooleanSupplier m_clawOpenSupplier;

    /**
     * Constructor.
     */
    public SendableArmPosition(BooleanSupplier clawOpenSupplier) {
      m_clawOpenSupplier = clawOpenSupplier;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("AlertsIdo");
      builder.addDoubleProperty("percentRaised", () -> 0.5, null);
      builder.addDoubleProperty("percentExtended", () -> 0.2, null);
      builder.addBooleanProperty("isClawOpen", m_clawOpenSupplier, null);
    }
  }

  /**
   * Constructor.
   */
  public ArmSystemSimWithWidgets(XboxController controller) {

    // FIRST, we call superclass
    super(controller);
  }

  private void addCommandButtons() {
    // Move to to middle node cone
    Shuffleboard.getTab("Simulation")
        .add(Constants.SimWidgets.kButtonMiddleNodeCone.m_name, new ArmToMiddleNodeCone(this))
        .withWidget(BuiltInWidgets.kCommand)
        .withPosition(Constants.SimWidgets.kButtonMiddleNodeCone.m_xpos,
            Constants.SimWidgets.kButtonMiddleNodeCone.m_ypos)
        .withSize(Constants.SimWidgets.kButtonMiddleNodeCone.m_width,
            Constants.SimWidgets.kButtonMiddleNodeCone.m_height);

    // Lower arm to ground
    Shuffleboard.getTab("Simulation")
        .add(Constants.SimWidgets.kButtonArmToGround.m_name, new ArmToGround(this))
        .withWidget(BuiltInWidgets.kCommand)
        .withPosition(Constants.SimWidgets.kButtonArmToGround.m_xpos,
            Constants.SimWidgets.kButtonArmToGround.m_ypos)
        .withSize(Constants.SimWidgets.kButtonArmToGround.m_width,
            Constants.SimWidgets.kButtonArmToGround.m_height);

    // Extend arm
    Shuffleboard.getTab("Simulation")
        .add(Constants.SimWidgets.kArmExtendFully.m_name, new ArmExtendFully(this))
        .withWidget(BuiltInWidgets.kCommand)
        .withPosition(Constants.SimWidgets.kArmExtendFully.m_xpos,
            Constants.SimWidgets.kArmExtendFully.m_ypos)
        .withSize(Constants.SimWidgets.kArmExtendFully.m_width,
            Constants.SimWidgets.kArmExtendFully.m_height);

    // Retract extender
    Shuffleboard.getTab("Simulation")
        .add(Constants.SimWidgets.kArmRetract.m_name, new RetractArmCommand(this))
        .withWidget(BuiltInWidgets.kCommand)
        .withPosition(Constants.SimWidgets.kArmRetract.m_xpos,
            Constants.SimWidgets.kArmRetract.m_ypos)
        .withSize(Constants.SimWidgets.kArmRetract.m_width,
            Constants.SimWidgets.kArmRetract.m_height);
  }

  private void addShuffleboardExtenderList() {
    // Extender functional
    Shuffleboard.getTab("Simulation")
        .addBoolean(Constants.SimWidgets.kExtenderFunctional.m_name,
            () -> !m_extenderSimulation.getIsBroken())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#8B0000"))
        .withPosition(Constants.SimWidgets.kExtenderFunctional.m_xpos,
            Constants.SimWidgets.kExtenderFunctional.m_ypos)
        .withSize(Constants.SimWidgets.kExtenderFunctional.m_width,
            Constants.SimWidgets.kExtenderFunctional.m_height);

    // Extender motor power
    Shuffleboard.getTab("Simulation")
        .addDouble(Constants.SimWidgets.kExtenderMotorPower.m_name,
            () -> m_extenderMotorOutputPercentage)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1.0, "max", 1.0, "show text", false))
        .withPosition(Constants.SimWidgets.kExtenderMotorPower.m_xpos,
            Constants.SimWidgets.kExtenderMotorPower.m_ypos)
        .withSize(Constants.SimWidgets.kExtenderMotorPower.m_width,
            Constants.SimWidgets.kExtenderMotorPower.m_height);

    // Extender percent extended
    Shuffleboard.getTab("Simulation")
        .addDouble(Constants.SimWidgets.kExtenderExtendedPercent.m_name,
            () -> m_extenderSimulation.getExtendedPercent())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 1.0, "show text", false))
        .withPosition(Constants.SimWidgets.kExtenderExtendedPercent.m_xpos,
            Constants.SimWidgets.kExtenderExtendedPercent.m_ypos)
        .withSize(Constants.SimWidgets.kExtenderExtendedPercent.m_width,
            Constants.SimWidgets.kExtenderExtendedPercent.m_height);

    // Extender sensor display
    Shuffleboard.getTab("Simulation")
        .addBoolean(Constants.SimWidgets.kExtenderSensor.m_name, () -> !m_sensorSim.getValue())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#FFFFFF"))
        .withPosition(Constants.SimWidgets.kExtenderSensor.m_xpos,
            Constants.SimWidgets.kExtenderSensor.m_ypos)
        .withSize(Constants.SimWidgets.kExtenderSensor.m_width,
            Constants.SimWidgets.kExtenderSensor.m_height);
  }

  private void addShuffleboardArmList() {
    // Arm functional display
    Shuffleboard.getTab("Simulation")
        .addBoolean(Constants.SimWidgets.kArmFunctional.m_name,
            () -> !m_armSimulation.getIsBroken())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#8B0000"))
        .withPosition(Constants.SimWidgets.kArmFunctional.m_xpos,
            Constants.SimWidgets.kArmFunctional.m_ypos)
        .withSize(Constants.SimWidgets.kArmFunctional.m_width,
            Constants.SimWidgets.kArmFunctional.m_height);

    // Arm position
    Shuffleboard.getTab("Simulation")
        .addDouble(Constants.SimWidgets.kArmPosition.m_name,
            () -> m_winchAbsoluteEncoder.getAbsolutePosition())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(Constants.SimWidgets.kArmPosition.m_xpos,
            Constants.SimWidgets.kArmPosition.m_ypos)
        .withSize(Constants.SimWidgets.kArmPosition.m_width,
            Constants.SimWidgets.kArmPosition.m_height);

    // Arm commands
    Shuffleboard.getTab("Simulation").add(Constants.SimWidgets.kArmSystemCommands.m_name, this)
        .withPosition(Constants.SimWidgets.kArmSystemCommands.m_xpos,
            Constants.SimWidgets.kArmSystemCommands.m_ypos)
        .withSize(Constants.SimWidgets.kArmSystemCommands.m_width,
            Constants.SimWidgets.kArmSystemCommands.m_height);
  }

  private void addShuffleboardWinchList() {
    // Winch functional display
    Shuffleboard.getTab("Simulation")
        .addBoolean(Constants.SimWidgets.kWinchFunctional.m_name,
            () -> !m_winchSimulation.getIsBroken())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#8B0000"))
        .withPosition(Constants.SimWidgets.kWinchFunctional.m_xpos,
            Constants.SimWidgets.kWinchFunctional.m_ypos)
        .withSize(Constants.SimWidgets.kWinchFunctional.m_width,
            Constants.SimWidgets.kWinchFunctional.m_height);

    // Winch motor power
    Shuffleboard.getTab("Simulation")
        .addDouble(Constants.SimWidgets.kWinchMotorPower.m_name, () -> m_winchMotorOutputPercentage)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1.0, "max", 1.0, "show text", false))
        .withPosition(Constants.SimWidgets.kWinchMotorPower.m_xpos,
            Constants.SimWidgets.kWinchMotorPower.m_ypos)
        .withSize(Constants.SimWidgets.kWinchMotorPower.m_width,
            Constants.SimWidgets.kWinchMotorPower.m_height);

    // Winch String % extended
    Shuffleboard.getTab("Simulation")
        .addDouble(Constants.SimWidgets.kWinchStringPercentExtended.m_name,
            () -> m_winchSimulation.getStringUnspooledPercent())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 1.0, "show text", false))
        .withPosition(Constants.SimWidgets.kWinchStringPercentExtended.m_xpos,
            Constants.SimWidgets.kWinchStringPercentExtended.m_ypos)
        .withSize(Constants.SimWidgets.kWinchStringPercentExtended.m_width,
            Constants.SimWidgets.kWinchStringPercentExtended.m_height);

    // Winch string location
    Shuffleboard.getTab("Simulation")
        .addString(Constants.SimWidgets.kWinchStringLocation.m_name,
            () -> m_winchSimulation.getWindingOrientationName())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(Constants.SimWidgets.kWinchStringLocation.m_xpos,
            Constants.SimWidgets.kWinchStringLocation.m_ypos)
        .withSize(Constants.SimWidgets.kWinchStringLocation.m_width,
            Constants.SimWidgets.kWinchStringLocation.m_height);
  }

  private void addShuffleboardWidgets() {
    addShuffleboardWinchList();
    addShuffleboardExtenderList();
    addShuffleboardArmList();

    // Add Robot Arm widget
    // $TODO Don't hardcode name of the widget and location
    Shuffleboard.getTab("Simulation")
        .add("Happy", new SendableArmPosition(() -> m_armSimulation.getGrabberOpen()))
        .withWidget("AlertsIdo").withPosition(7, 0).withSize(3, 3);
  }

  @Override
  public void initDashBoard() {
    super.initDashBoard();

    addShuffleboardWidgets();
    addCommandButtons();
  }
}
