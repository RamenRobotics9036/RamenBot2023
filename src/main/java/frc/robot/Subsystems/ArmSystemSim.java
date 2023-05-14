package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;
import java.util.function.BooleanSupplier;

import frc.robot.Constants;
import frc.robot.Commands.ArmToMiddleNodeCone;
import frc.robot.Commands.ArmToGround;
import frc.robot.Simulation.ArmSimulation;
import frc.robot.Simulation.WinchSimulation;
import frc.robot.Simulation.WinchSimulation.StringOrientation;
import frc.robot.Simulation.ExtenderSimulation;

public class ArmSystemSim extends ArmSystem {
  private DutyCycleEncoderSim m_winchAbsoluteEncoderSim;

  private DCMotor m_winchMotorModel;
  private DCMotorSim m_winchMotorSim;
  private RelativeEncoderSim m_winchEncoderSim;
  private double m_winchMotorOutputPercentage = 0;
  private WinchSimulation m_WinchSimulation;

  private DCMotor m_extenderMotorModel;
  private DCMotorSim m_extenderMotorSim;
  private RelativeEncoderSim m_extenderEncoderSim;
  private double m_extenderMotorOutputPercentage = 0;
  private ExtenderSimulation m_ExtenderSimulation;

  private DIOSim m_sensorSim;

  private ArmSimulation m_ArmSimulation;
  private BooleanSupplier m_grabberOpenSupplier = null;

  public static ArmSystem CreateArmSystemInstance(int armWinchChannel, int armExtenderChannel,
      XboxController m_controller, double m_deadband, boolean squareInputs, double maxOutputWinch) {
    ArmSystem result;

    if (RobotBase.isSimulation()) {
      result = new ArmSystemSim(
          armWinchChannel,
          armExtenderChannel,
          m_controller,
          m_deadband,
          squareInputs,
          maxOutputWinch);

      System.out.println("ARMSYSTEM: **** Simulation ****");

    } else {
      result = new ArmSystem(
          armWinchChannel,
          armExtenderChannel,
          m_controller,
          m_deadband,
          squareInputs,
          maxOutputWinch);

      System.out.println("ARMSYSTEM: Physical Robot version");
    }

    return result;
  }

  // Constructor
  public ArmSystemSim(int armWinchChannel, int armExtenderChannel, XboxController m_controller, double m_deadband,
      boolean squareInputs, double maxOutputWinch) {
    // FIRST, we call superclass
    super(armWinchChannel,
        armExtenderChannel,
        m_controller,
        m_deadband,
        squareInputs,
        maxOutputWinch);

    // This entire class should only be instantiated when we're under simulation.
    // But just in-case someone tries to instantiate it otherwise, we do an extra
    // check here.
    if (!RobotBase.isSimulation()) {
      return;
    }

    CreateWinchSimParts();
    CreateExtenderSimParts();

    m_sensorSim = new DIOSim(sensor);

    // Create simulated absolute encoder
    m_winchAbsoluteEncoderSim = new DutyCycleEncoderSim2(m_winchAbsoluteEncoder);

    m_ArmSimulation = new ArmSimulation(
        m_WinchSimulation,
        m_winchAbsoluteEncoderSim,
        Constants.OperatorConstants.kWinchEncoderUpperLimit,
        Constants.OperatorConstants.kWinchEncoderLowerLimit,
        Constants.SimConstants.kdeltaRotationsBeforeBroken,
        Constants.SimConstants.kgrabberBreaksIfOpenBelowThisLimit,
        Constants.SimConstants.karmHeightFromWinchToPivotPoint,
        Constants.SimConstants.karmLengthFromEdgeToPivot,
        Constants.SimConstants.karmLengthFromEdgeToPivot_Min,
        Constants.SimConstants.karmEncoderRotationsOffset);
  }

  private void CreateWinchSimParts() {
    // Model a NEO motor (or any other motor)
    m_winchMotorModel = DCMotor.getNEO(1); // 1 motor in the gearbox

    // Create the motor simulation with motor model, gear ratio, and moment of
    // inertia
    double motorMomentInertia = 0.0005;
    m_winchMotorSim = new DCMotorSim(
        m_winchMotorModel,
        Constants.SimConstants.kwinchSimGearRatio,
        motorMomentInertia);

    // Create winch simulated encoder
    m_winchEncoderSim = new RelativeEncoderSim(m_winchEncoder);

    m_WinchSimulation = new WinchSimulation(
        m_winchEncoderSim,
        0.0254, // Spool diameter (1 inch)
        Constants.SimConstants.kTotalStringLenMeters,
        Constants.SimConstants.kCurrentLenSpooled,
        StringOrientation.BackOfRobot,
        true); // invert motor for winch
  }

  private void CreateExtenderSimParts() {
    // Model a NEO motor (or any other motor)
    m_extenderMotorModel = DCMotor.getNEO(1); // 1 motor in the gearbox

    // Create the motor simulation with motor model, gear ratio, and moment of
    // inertia
    double motorMomentInertia = 0.0005;
    m_extenderMotorSim = new DCMotorSim(
        m_extenderMotorModel,
        Constants.SimConstants.kextenderSimGearRatio,
        motorMomentInertia);

    // Create extender simulated encoder
    m_extenderEncoderSim = new RelativeEncoderSim(m_extenderEncoder);

    m_ExtenderSimulation = new ExtenderSimulation(
        m_extenderEncoderSim,
        Constants.SimConstants.kcylinderDiameterMeters,
        Constants.SimConstants.kTotalExtenderLenMeters,
        Constants.SimConstants.kInitialExtendedLen,
        true);
  }

  private void AddCommandButtons() {
    // Create a list layout
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kButtonList.name, BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "HIDDEN"))
        .withPosition(Constants.SimWidgets.kButtonList.x, Constants.SimWidgets.kButtonList.y)
        .withSize(Constants.SimWidgets.kButtonList.width, Constants.SimWidgets.kButtonList.height);

    // Move to to middle node cone
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kButtonList.name, BuiltInLayouts.kList)
        .add(Constants.SimWidgets.kButtonMiddleNodeCone, new ArmToMiddleNodeCone(this))
        .withWidget(BuiltInWidgets.kCommand);

    // Lower arm to ground
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kButtonList.name, BuiltInLayouts.kList)
        .add(Constants.SimWidgets.kButtonArmToGround, new ArmToGround(this))
        .withWidget(BuiltInWidgets.kCommand);        
  }

  private void AddShuffleboardExtenderList() {
      // Create a list layout
      Shuffleboard.getTab("Simulation")
          .getLayout(Constants.SimWidgets.kExtenderList.name, BuiltInLayouts.kList)
          .withProperties(Map.of("Label position", "TOP"))
          .withPosition(Constants.SimWidgets.kExtenderList.x, Constants.SimWidgets.kExtenderList.y)
          .withSize(Constants.SimWidgets.kExtenderList.width, Constants.SimWidgets.kExtenderList.height);

    // Extender functional
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kExtenderList.name, BuiltInLayouts.kList)
        .addBoolean(Constants.SimWidgets.kExtenderFunctional, () -> !m_ExtenderSimulation.GetIsBroken())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#8B0000"));

    // Extender motor power
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kExtenderList.name, BuiltInLayouts.kList)
        .addDouble(Constants.SimWidgets.kExtenderMotorPower, () -> m_extenderMotorOutputPercentage)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of(
            "min", -1.0,
            "max", 1.0,
            "show text", false));

    // Extender percent extended
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kExtenderList.name, BuiltInLayouts.kList)
        .addDouble(Constants.SimWidgets.kExtenderExtendedPercent, () -> m_ExtenderSimulation.GetExtendedPercent())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of(
            "min", 0.0,
            "max", 1.0,
            "show text", false));

    // Extender sensor display
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kExtenderList.name, BuiltInLayouts.kList)
        .addBoolean(Constants.SimWidgets.kExtenderSensor, () -> !m_sensorSim.getValue())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#FFFFFF"));
  }

  private void AddShuffleboardArmList() {
    // Create a list layout
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kArmList.name, BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"))
        .withPosition(Constants.SimWidgets.kArmList.x, Constants.SimWidgets.kArmList.y)
        .withSize(Constants.SimWidgets.kArmList.width, Constants.SimWidgets.kArmList.height);

    // Arm functional display
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kArmList.name, BuiltInLayouts.kList)
        .addBoolean(Constants.SimWidgets.kArmFunctional, () -> !m_ArmSimulation.GetIsBroken())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#8B0000"));

    // Arm position
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kArmList.name, BuiltInLayouts.kList)
        .addDouble(Constants.SimWidgets.kArmPosition, () -> m_winchAbsoluteEncoder.getAbsolutePosition())
        .withWidget(BuiltInWidgets.kTextView);

    // Arm commands
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kArmList.name, BuiltInLayouts.kList)
        .add(Constants.SimWidgets.kArmSystemCommands, this);
  }

  private void AddShuffleboardWinchList() {
    // Create a list layout
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kWinchList.name, BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"))
        .withPosition(Constants.SimWidgets.kWinchList.x, Constants.SimWidgets.kWinchList.y)
        .withSize(Constants.SimWidgets.kWinchList.width, Constants.SimWidgets.kWinchList.height);

    // Winch functional display
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kWinchList.name, BuiltInLayouts.kList)
        .addBoolean(Constants.SimWidgets.kWinchFunctional, () -> !m_WinchSimulation.GetIsBroken())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#8B0000"));

    // Winch motor power
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kWinchList.name, BuiltInLayouts.kList)
        .addDouble(Constants.SimWidgets.kWinchMotorPower, () -> m_winchMotorOutputPercentage)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of(
            "min", -1.0,
            "max", 1.0,
            "show text", false));

    // Winch String % extended
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kWinchList.name, BuiltInLayouts.kList)
        .addDouble(Constants.SimWidgets.kWinchStringPercentExtended, () -> m_WinchSimulation.GetStringExtendedPercent())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of(
            "min", 0.0,
            "max", 1.0,
            "show text", false));

    // Winch string location
    Shuffleboard.getTab("Simulation")
        .getLayout(Constants.SimWidgets.kWinchList.name, BuiltInLayouts.kList)
        .addString(Constants.SimWidgets.kWinchStringLocation, () -> m_WinchSimulation.GetStringOrientationName())
        .withWidget(BuiltInWidgets.kTextView);
  }

  private void AddShuffleboardWidgets() {
    AddShuffleboardWinchList();
    AddShuffleboardExtenderList();
    AddShuffleboardArmList();
  }

  private boolean isRobotEnabled() {
    return RobotState.isEnabled();
  }

  public void setGrabberOpenSupplier(BooleanSupplier grabberOpenSupplier) {
    m_grabberOpenSupplier = grabberOpenSupplier;
  }

  @Override
  public void initDashBoard() {
    super.initDashBoard();

    AddShuffleboardWidgets();
    AddCommandButtons();
  }

  @Override
  public void periodic() {
    super.periodic();

    // $TODO - This will move into ArmSimulation
    // if (m_grabberOpenSupplier != null) {
    // System.out.println(m_grabberOpenSupplier.getAsBoolean());
    // }

    // When Robot is disabled, the entire simulation freezes
    if (isRobotEnabled()) {
      m_WinchSimulation.periodic();
      m_ExtenderSimulation.periodic();
      m_ArmSimulation.periodic();
    }
  }

  private static void UpdateSimMotorPosition(double motorOutputPercentage,
      DCMotorSim motorSim, RelativeEncoderSim encoderSim) {
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

      UpdateSimMotorPosition(
          m_winchMotorOutputPercentage,
          m_winchMotorSim,
          m_winchEncoderSim);

      // Get the EXTENDER motor controller output percentage
      m_extenderMotorOutputPercentage = m_armExtender.get();

      UpdateSimMotorPosition(
          m_extenderMotorOutputPercentage,
          m_extenderMotorSim,
          m_extenderEncoderSim);

      m_WinchSimulation.simulationPeriodic();
      m_ExtenderSimulation.simulationPeriodic();
      m_ArmSimulation.simulationPeriodic();

      boolean isExtenderSensorOn = m_ExtenderSimulation.GetExtendedLen() <= Constants.SimConstants.kextenderFullyRetractedLen;
      m_sensorSim.setValue(!isExtenderSensorOn);
    }
  }
}
