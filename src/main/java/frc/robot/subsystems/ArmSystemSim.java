package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.Constants;
import frc.robot.simulation.ArmSimulation;
import frc.robot.simulation.ExtenderSimulation;
import frc.robot.simulation.WinchSimulation;
import frc.robot.simulation.WinchSimulation.WindingOrientation;
import frc.robot.simulation.framework.SimManagerInterface;
import frc.robot.simulation.motor.MotorSimInput;
import frc.robot.simulation.motor.MotorSimManager;
import frc.robot.simulation.motor.MotorSimOutput;
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
  protected double m_winchMotorOutputPercentage = 0;
  protected WinchSimulation m_winchSimulation;

  private RelativeEncoderSim m_extenderEncoderSim;
  private SimManagerInterface<Double, Double> m_extenderMotorSimManager;
  protected ExtenderSimulation m_extenderSimulation;

  protected DIOSim m_sensorSim;

  protected ArmSimulation m_armSimulation;

  /**
   * Creates an instance of the ArmSystem or ArmSystemSim class.
   */
  public static ArmSystem createArmSystemInstance(XboxController controller) {
    ArmSystem result;

    if (RobotBase.isSimulation()) {
      result = new ArmSystemSimWithWidgets(controller);

      System.out.println("ARMSYSTEM: **** Simulation ****");

    }
    else {
      result = new ArmSystem(controller);

      System.out.println("ARMSYSTEM: Physical Robot version");
    }

    return result;
  }

  /**
   * Constructor.
   */
  public ArmSystemSim(XboxController controller) {

    // FIRST, we call superclass
    super(controller);

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
    // Create extender simulated encoder
    m_extenderEncoderSim = new RelativeEncoderSim(m_extenderEncoder);

    // Create the motor simulation for the extender
    m_extenderMotorSimManager = new MotorSimManager();
    m_extenderMotorSimManager.setInputHandler(new MotorSimInput(m_armExtender));
    m_extenderMotorSimManager.setOutputHandler(new MotorSimOutput(m_extenderEncoderSim));

    m_extenderSimulation = new ExtenderSimulation(m_extenderEncoderSim,
        Constants.SimConstants.kcylinderDiameterMeters,
        Constants.SimConstants.kTotalExtenderLenMeters, Constants.SimConstants.kInitialExtendedLen,
        true);
  }

  // $TODO Get rid of isRobotEnabled
  private boolean isRobotEnabled() {
    return RobotState.isEnabled();
  }

  public void setGrabberOpenSupplier(BooleanSupplier grabberOpenSupplier) {
    m_armSimulation.setGrabberOpenSupplier(grabberOpenSupplier);
  }

  @Override
  public void periodic() {
    super.periodic();

    // When Robot is disabled, the entire simulation freezes
    // $TODO Why are these simulations done here, and not in simulationPeriodic()?
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

      m_extenderMotorSimManager.simulationPeriodic();

      m_winchSimulation.simulationPeriodic();
      m_extenderSimulation.simulationPeriodic();
      m_armSimulation.simulationPeriodic();

      boolean isExtenderSensorOn = m_extenderSimulation
          .getExtendedLen() <= Constants.SimConstants.kextenderFullyRetractedLen;
      m_sensorSim.setValue(!isExtenderSensorOn);
    }
  }
}
