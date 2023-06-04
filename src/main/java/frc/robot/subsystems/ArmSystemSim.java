package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.Constants;
import frc.robot.commands.ArmExtendFully;
import frc.robot.commands.ArmToGround;
import frc.robot.commands.ArmToMiddleNodeCone;
import frc.robot.commands.RetractArmCommand;
import frc.robot.simulation.ArmSimulation;
import frc.robot.simulation.ExtenderSimulation;
import frc.robot.simulation.WinchSimulation;
import frc.robot.simulation.WinchSimulation.WindingOrientation;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Subclass of ArmSystem that is used for simulation. Note that this code isn't run if
 * the robot is not running in simulation mode.
 */
public class ArmSystemSim extends ArmSystem {
  private DutyCycleEncoderSim m_winchAbsoluteEncoderSim;

  private DCMotor m_winchMotorModel;
  private DCMotorSim m_winchMotorSim;
  private RelativeEncoderSim m_winchEncoderSim;
  private double m_winchMotorOutputPercentage = 0;
  private WinchSimulation m_winchSimulation;

  private DCMotor m_extenderMotorModel;
  private DCMotorSim m_extenderMotorSim;
  private RelativeEncoderSim m_extenderEncoderSim;
  private double m_extenderMotorOutputPercentage = 0;
  private ExtenderSimulation m_extenderSimulation;

  private DIOSim m_sensorSim;

  private ArmSimulation m_armSimulation;

  /**
   * Creates an instance of the ArmSystem or ArmSystemSim class.
   */
  public static ArmSystem createArmSystemInstance(XboxController controller,
      boolean squareInputs,
      double maxOutputWinch) {
    ArmSystem result;

    if (RobotBase.isSimulation()) {
      result = new ArmSystemSim(controller, squareInputs, maxOutputWinch);

      System.out.println("ARMSYSTEM: **** Simulation ****");

    }
    else {
      result = new ArmSystem(controller, squareInputs, maxOutputWinch);

      System.out.println("ARMSYSTEM: Physical Robot version");
    }

    return result;
  }

  /**
   * Constructor.
   */
  public ArmSystemSim(XboxController controller, boolean squareInputs, double maxOutputWinch) {

    // FIRST, we call superclass
    super(controller, squareInputs, maxOutputWinch);

    // This entire class should only be instantiated when we're under simulation.
    // But just in-case someone tries to instantiate it otherwise, we do an extra
    // check here.
    if (!RobotBase.isSimulation()) {
      return;
    }

    createWinchSimParts();
    createExtenderSimParts();

    m_sensorSim = new DIOSim(m_sensor);

    // Create simulated absolute encoder
    m_winchAbsoluteEncoderSim = new DutyCycleEncoderSim2(m_winchAbsoluteEncoder);

    m_armSimulation = new ArmSimulation(m_winchSimulation, m_winchAbsoluteEncoderSim,
        Constants.OperatorConstants.kWinchEncoderUpperLimit,
        Constants.OperatorConstants.kWinchEncoderLowerLimit,
        Constants.SimConstants.kdeltaRotationsBeforeBroken,
        Constants.SimConstants.kgrabberBreaksIfOpenBelowThisLimit,
        Constants.SimConstants.karmHeightFromWinchToPivotPoint,
        Constants.SimConstants.karmLengthFromEdgeToPivot,
        Constants.SimConstants.klengthFromPivotPointToArmBackEnd_Min,
        Constants.SimConstants.karmEncoderRotationsOffset);
  }

  private void createWinchSimParts() {
    // Model a NEO motor (or any other motor)
    m_winchMotorModel = DCMotor.getNEO(1); // 1 motor in the gearbox

    // Create the motor simulation with motor model, gear ratio, and moment of
    // inertia
    double motorMomentInertia = 0.0005;
    m_winchMotorSim = new DCMotorSim(m_winchMotorModel, Constants.SimConstants.kwinchSimGearRatio,
        motorMomentInertia);

    // Create winch simulated encoder
    m_winchEncoderSim = new RelativeEncoderSim(m_winchEncoder);

    m_winchSimulation = new WinchSimulation(m_winchEncoderSim, 0.0254, // Spool diameter (1 inch)
        Constants.SimConstants.kTotalStringLenMeters, Constants.SimConstants.kCurrentLenSpooled,
        WindingOrientation.BackOfRobot, true); // invert motor for winch
  }

  private void createExtenderSimParts() {
    // Model a NEO motor (or any other motor)
    m_extenderMotorModel = DCMotor.getNEO(1); // 1 motor in the gearbox

    // Create the motor simulation with motor model, gear ratio, and moment of
    // inertia
    double motorMomentInertia = 0.0005;
    m_extenderMotorSim = new DCMotorSim(m_extenderMotorModel,
        Constants.SimConstants.kextenderSimGearRatio, motorMomentInertia);

    // Create extender simulated encoder
    m_extenderEncoderSim = new RelativeEncoderSim(m_extenderEncoder);

    m_extenderSimulation = new ExtenderSimulation(m_extenderEncoderSim,
        Constants.SimConstants.kcylinderDiameterMeters,
        Constants.SimConstants.kTotalExtenderLenMeters, Constants.SimConstants.kInitialExtendedLen,
        true);
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
  }

  private boolean isRobotEnabled() {
    return RobotState.isEnabled();
  }

  public void setGrabberOpenSupplier(BooleanSupplier grabberOpenSupplier) {
    m_armSimulation.setGrabberOpenSupplier(grabberOpenSupplier);
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
      m_winchSimulation.periodic();
      m_extenderSimulation.periodic();
      m_armSimulation.periodic();
    }
  }

  private static void updateSimMotorPosition(double motorOutputPercentage,
      DCMotorSim motorSim,
      RelativeEncoderSim encoderSim) {
    // Calculate the input voltage for the motor
    double inputVoltageVolts = motorOutputPercentage * 12.0;

    // Update the motor simulation
    motorSim.setInput(inputVoltageVolts);
    motorSim.update(0.02);

    // Update the Encoder based on the simulation - the units are "number of
    // rotations"
    double motorRotations = motorSim.getAngularPositionRotations();
    encoderSim.setPosition(motorRotations);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // When Robot is disabled, the entire simulation freezes
    if (isRobotEnabled()) {

      // Get the WINCH motor controller output percentage
      m_winchMotorOutputPercentage = m_armWinch.get();

      updateSimMotorPosition(m_winchMotorOutputPercentage, m_winchMotorSim, m_winchEncoderSim);

      // Get the EXTENDER motor controller output percentage
      m_extenderMotorOutputPercentage = m_armExtender.get();

      updateSimMotorPosition(m_extenderMotorOutputPercentage,
          m_extenderMotorSim,
          m_extenderEncoderSim);

      m_winchSimulation.simulationPeriodic();
      m_extenderSimulation.simulationPeriodic();
      m_armSimulation.simulationPeriodic();

      boolean isExtenderSensorOn = m_extenderSimulation
          .getExtendedLen() <= Constants.SimConstants.kextenderFullyRetractedLen;
      m_sensorSim.setValue(!isExtenderSensorOn);
    }
  }
}
