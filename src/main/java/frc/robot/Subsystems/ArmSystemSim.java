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
import frc.robot.Commands.RetractArmCommand;
import frc.robot.Commands.ArmExtendFully;
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

  public static ArmSystem CreateArmSystemInstance(int armWinchChannel,
      int armExtenderChannel,
      XboxController m_controller,
      double m_deadband,
      boolean squareInputs,
      double maxOutputWinch) {
    ArmSystem result;

    if (RobotBase.isSimulation()) {
      result = new ArmSystemSim(armWinchChannel, armExtenderChannel, m_controller, m_deadband,
          squareInputs, maxOutputWinch);

      System.out.println("ARMSYSTEM: **** Simulation ****");

    }
    else {
      result = new ArmSystem(armWinchChannel, armExtenderChannel, m_controller, m_deadband,
          squareInputs, maxOutputWinch);

      System.out.println("ARMSYSTEM: Physical Robot version");
    }

    return result;
  }

  // Constructor
  public ArmSystemSim(int armWinchChannel,
      int armExtenderChannel,
      XboxController m_controller,
      double m_deadband,
      boolean squareInputs,
      double maxOutputWinch) {
    // FIRST, we call superclass
    super(armWinchChannel, armExtenderChannel, m_controller, m_deadband, squareInputs,
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

    m_ArmSimulation = new ArmSimulation(m_WinchSimulation, m_winchAbsoluteEncoderSim,
        Constants.OperatorConstants.kWinchEncoderUpperLimit,
        Constants.OperatorConstants.kWinchEncoderLowerLimit,
        Constants.SimConstants.kdeltaRotationsBeforeBroken,
        Constants.SimConstants.kgrabberBreaksIfOpenBelowThisLimit,
        Constants.SimConstants.karmHeightFromWinchToPivotPoint,
        Constants.SimConstants.karmLengthFromEdgeToPivot,
        Constants.SimConstants.klengthFromPivotPointToArmBackEnd_Min,
        Constants.SimConstants.karmEncoderRotationsOffset);
  }

  private void CreateWinchSimParts() {
    // Model a NEO motor (or any other motor)
    m_winchMotorModel = DCMotor.getNEO(1); // 1 motor in the gearbox

    // Create the motor simulation with motor model, gear ratio, and moment of
    // inertia
    double motorMomentInertia = 0.0005;
    m_winchMotorSim = new DCMotorSim(m_winchMotorModel, Constants.SimConstants.kwinchSimGearRatio,
        motorMomentInertia);

    // Create winch simulated encoder
    m_winchEncoderSim = new RelativeEncoderSim(m_winchEncoder);

    m_WinchSimulation = new WinchSimulation(m_winchEncoderSim, 0.0254, // Spool diameter (1 inch)
        Constants.SimConstants.kTotalStringLenMeters, Constants.SimConstants.kCurrentLenSpooled,
        StringOrientation.BackOfRobot, true); // invert motor for winch
  }

  private void CreateExtenderSimParts() {
    // Model a NEO motor (or any other motor)
    m_extenderMotorModel = DCMotor.getNEO(1); // 1 motor in the gearbox

    // Create the motor simulation with motor model, gear ratio, and moment of
    // inertia
    double motorMomentInertia = 0.0005;
    m_extenderMotorSim = new DCMotorSim(m_extenderMotorModel,
        Constants.SimConstants.kextenderSimGearRatio, motorMomentInertia);

    // Create extender simulated encoder
    m_extenderEncoderSim = new RelativeEncoderSim(m_extenderEncoder);

    m_ExtenderSimulation = new ExtenderSimulation(m_extenderEncoderSim,
        Constants.SimConstants.kcylinderDiameterMeters,
        Constants.SimConstants.kTotalExtenderLenMeters, Constants.SimConstants.kInitialExtendedLen,
        true);
  }

  private void AddCommandButtons() {
    // Move to to middle node cone
    Shuffleboard.getTab("Simulation")
        .add(Constants.SimWidgets.kButtonMiddleNodeCone.name, new ArmToMiddleNodeCone(this))
        .withWidget(BuiltInWidgets.kCommand)
        .withPosition(Constants.SimWidgets.kButtonMiddleNodeCone.x,
            Constants.SimWidgets.kButtonMiddleNodeCone.y)
        .withSize(Constants.SimWidgets.kButtonMiddleNodeCone.width,
            Constants.SimWidgets.kButtonMiddleNodeCone.height);

    // Lower arm to ground
    Shuffleboard.getTab("Simulation")
        .add(Constants.SimWidgets.kButtonArmToGround.name, new ArmToGround(this))
        .withWidget(BuiltInWidgets.kCommand)
        .withPosition(Constants.SimWidgets.kButtonArmToGround.x,
            Constants.SimWidgets.kButtonArmToGround.y)
        .withSize(Constants.SimWidgets.kButtonArmToGround.width,
            Constants.SimWidgets.kButtonArmToGround.height);

    // Extend arm
    Shuffleboard.getTab("Simulation")
        .add(Constants.SimWidgets.kArmExtendFully.name, new ArmExtendFully(this))
        .withWidget(BuiltInWidgets.kCommand)
        .withPosition(Constants.SimWidgets.kArmExtendFully.x,
            Constants.SimWidgets.kArmExtendFully.y)
        .withSize(Constants.SimWidgets.kArmExtendFully.width,
            Constants.SimWidgets.kArmExtendFully.height);

    // Retract extender
    Shuffleboard.getTab("Simulation")
        .add(Constants.SimWidgets.kArmRetract.name, new RetractArmCommand(this))
        .withWidget(BuiltInWidgets.kCommand)
        .withPosition(Constants.SimWidgets.kArmRetract.x, Constants.SimWidgets.kArmRetract.y)
        .withSize(Constants.SimWidgets.kArmRetract.width, Constants.SimWidgets.kArmRetract.height);
  }

  private void AddShuffleboardExtenderList() {
    // Extender functional
    Shuffleboard.getTab("Simulation")
        .addBoolean(Constants.SimWidgets.kExtenderFunctional.name,
            () -> !m_ExtenderSimulation.GetIsBroken())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#8B0000"))
        .withPosition(Constants.SimWidgets.kExtenderFunctional.x,
            Constants.SimWidgets.kExtenderFunctional.y)
        .withSize(Constants.SimWidgets.kExtenderFunctional.width,
            Constants.SimWidgets.kExtenderFunctional.height);

    // Extender motor power
    Shuffleboard.getTab("Simulation")
        .addDouble(Constants.SimWidgets.kExtenderMotorPower.name,
            () -> m_extenderMotorOutputPercentage)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1.0, "max", 1.0, "show text", false))
        .withPosition(Constants.SimWidgets.kExtenderMotorPower.x,
            Constants.SimWidgets.kExtenderMotorPower.y)
        .withSize(Constants.SimWidgets.kExtenderMotorPower.width,
            Constants.SimWidgets.kExtenderMotorPower.height);

    // Extender percent extended
    Shuffleboard.getTab("Simulation")
        .addDouble(Constants.SimWidgets.kExtenderExtendedPercent.name,
            () -> m_ExtenderSimulation.GetExtendedPercent())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 1.0, "show text", false))
        .withPosition(Constants.SimWidgets.kExtenderExtendedPercent.x,
            Constants.SimWidgets.kExtenderExtendedPercent.y)
        .withSize(Constants.SimWidgets.kExtenderExtendedPercent.width,
            Constants.SimWidgets.kExtenderExtendedPercent.height);

    // Extender sensor display
    Shuffleboard.getTab("Simulation")
        .addBoolean(Constants.SimWidgets.kExtenderSensor.name, () -> !m_sensorSim.getValue())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#FFFFFF"))
        .withPosition(Constants.SimWidgets.kExtenderSensor.x,
            Constants.SimWidgets.kExtenderSensor.y)
        .withSize(Constants.SimWidgets.kExtenderSensor.width,
            Constants.SimWidgets.kExtenderSensor.height);
  }

  private void AddShuffleboardArmList() {
    // Arm functional display
    Shuffleboard.getTab("Simulation")
        .addBoolean(Constants.SimWidgets.kArmFunctional.name, () -> !m_ArmSimulation.GetIsBroken())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#8B0000"))
        .withPosition(Constants.SimWidgets.kArmFunctional.x, Constants.SimWidgets.kArmFunctional.y)
        .withSize(Constants.SimWidgets.kArmFunctional.width,
            Constants.SimWidgets.kArmFunctional.height);

    // Arm position
    Shuffleboard.getTab("Simulation")
        .addDouble(Constants.SimWidgets.kArmPosition.name,
            () -> m_winchAbsoluteEncoder.getAbsolutePosition())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(Constants.SimWidgets.kArmPosition.x, Constants.SimWidgets.kArmPosition.y)
        .withSize(Constants.SimWidgets.kArmPosition.width,
            Constants.SimWidgets.kArmPosition.height);

    // Arm commands
    Shuffleboard.getTab("Simulation").add(Constants.SimWidgets.kArmSystemCommands.name, this)
        .withPosition(Constants.SimWidgets.kArmSystemCommands.x,
            Constants.SimWidgets.kArmSystemCommands.y)
        .withSize(Constants.SimWidgets.kArmSystemCommands.width,
            Constants.SimWidgets.kArmSystemCommands.height);
  }

  private void AddShuffleboardWinchList() {
    // Winch functional display
    Shuffleboard.getTab("Simulation")
        .addBoolean(Constants.SimWidgets.kWinchFunctional.name,
            () -> !m_WinchSimulation.GetIsBroken())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#C0FBC0", "colorWhenFalse", "#8B0000"))
        .withPosition(Constants.SimWidgets.kWinchFunctional.x,
            Constants.SimWidgets.kWinchFunctional.y)
        .withSize(Constants.SimWidgets.kWinchFunctional.width,
            Constants.SimWidgets.kWinchFunctional.height);

    // Winch motor power
    Shuffleboard.getTab("Simulation")
        .addDouble(Constants.SimWidgets.kWinchMotorPower.name, () -> m_winchMotorOutputPercentage)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1.0, "max", 1.0, "show text", false))
        .withPosition(Constants.SimWidgets.kWinchMotorPower.x,
            Constants.SimWidgets.kWinchMotorPower.y)
        .withSize(Constants.SimWidgets.kWinchMotorPower.width,
            Constants.SimWidgets.kWinchMotorPower.height);

    // Winch String % extended
    Shuffleboard.getTab("Simulation")
        .addDouble(Constants.SimWidgets.kWinchStringPercentExtended.name,
            () -> m_WinchSimulation.GetStringExtendedPercent())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 1.0, "show text", false))
        .withPosition(Constants.SimWidgets.kWinchStringPercentExtended.x,
            Constants.SimWidgets.kWinchStringPercentExtended.y)
        .withSize(Constants.SimWidgets.kWinchStringPercentExtended.width,
            Constants.SimWidgets.kWinchStringPercentExtended.height);

    // Winch string location
    Shuffleboard.getTab("Simulation")
        .addString(Constants.SimWidgets.kWinchStringLocation.name,
            () -> m_WinchSimulation.GetStringOrientationName())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(Constants.SimWidgets.kWinchStringLocation.x,
            Constants.SimWidgets.kWinchStringLocation.y)
        .withSize(Constants.SimWidgets.kWinchStringLocation.width,
            Constants.SimWidgets.kWinchStringLocation.height);
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
    m_ArmSimulation.setGrabberOpenSupplier(grabberOpenSupplier);
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

    // When Robot is disabled, the entire simulation freezes
    if (isRobotEnabled()) {
      m_WinchSimulation.periodic();
      m_ExtenderSimulation.periodic();
      m_ArmSimulation.periodic();
    }
  }

  private static void UpdateSimMotorPosition(double motorOutputPercentage,
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

      UpdateSimMotorPosition(m_winchMotorOutputPercentage, m_winchMotorSim, m_winchEncoderSim);

      // Get the EXTENDER motor controller output percentage
      m_extenderMotorOutputPercentage = m_armExtender.get();

      UpdateSimMotorPosition(m_extenderMotorOutputPercentage,
          m_extenderMotorSim,
          m_extenderEncoderSim);

      m_WinchSimulation.simulationPeriodic();
      m_ExtenderSimulation.simulationPeriodic();
      m_ArmSimulation.simulationPeriodic();

      boolean isExtenderSensorOn = m_ExtenderSimulation
          .GetExtendedLen() <= Constants.SimConstants.kextenderFullyRetractedLen;
      m_sensorSim.setValue(!isExtenderSensorOn);
    }
  }
}
