package frc.robot.simulation;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.RelativeEncoderSim;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.subsystems.DutyCycleEncoderSim2;
import frc.robot.simulation.WinchSimulation.WindingOrientation;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.BooleanSupplier;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import frc.robot.Constants;

public class ArmSimulationTest {
  private final double m_armTopRotationsLimit = 0.25;
  private final double m_armBottomRotationsLimit = 0.75;
  private final double m_armDeltaRotationsBeforeBroken = 0;
  private final double m_grabberBreaksIfOpenBelowThisLimit = 0.80;
  private final double m_winchSpoolDiameterMeters = 0.01; // (1 centimeter)
  private final double m_winchTotalStringLenMeters = 5;
  private final double m_winchInitialLenSpooled = 4;
  private final WindingOrientation m_winchInitialStringOrientation = WindingOrientation.BackOfRobot;
  private final boolean m_winchinvertMotor = false;
  private final double m_armHeightFromWinchToPivotPoint = 1;
  private final double m_armLengthFromEdgeToPivot = 0.5; // The pivot is halfway down the arm
  private final double m_armLengthFromEdgeToPivot_Min = 0.1;
  private final double m_encoderPositionOffsetRotations = 0;

  private WinchSimulation m_winchSimulation;
  private RelativeEncoderSim m_winchRelEncoderSim;
  private DutyCycleEncoder m_winchAbsoluteEncoder = null;
  private DutyCycleEncoderSim m_winchAbsoluteEncoderSim = null;

  @BeforeEach
  public void setUp() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    m_winchRelEncoderSim = new RelativeEncoderSim(null, true); // test-mode

    m_winchSimulation = new WinchSimulation(m_winchRelEncoderSim, m_winchSpoolDiameterMeters,
        m_winchTotalStringLenMeters, m_winchInitialLenSpooled, m_winchInitialStringOrientation,
        m_winchinvertMotor);

    m_winchAbsoluteEncoder = new DutyCycleEncoder(
        Constants.OperatorConstants.kAbsoluteEncoderWinchChannel);
    m_winchAbsoluteEncoderSim = new DutyCycleEncoderSim2(m_winchAbsoluteEncoder);
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  void shutdown() throws Exception {
    if (m_winchAbsoluteEncoder != null) {
      m_winchAbsoluteEncoder.close();
      m_winchAbsoluteEncoder = null;
    }
  }

  @Test
  public void CreateArmSimulationShouldSucceed() {
    ArmSimulation tempArmSimulation = new ArmSimulation(m_winchSimulation,
        m_winchAbsoluteEncoderSim, m_armTopRotationsLimit, m_armBottomRotationsLimit,
        m_armDeltaRotationsBeforeBroken, m_grabberBreaksIfOpenBelowThisLimit,
        m_armHeightFromWinchToPivotPoint, m_armLengthFromEdgeToPivot,
        m_armLengthFromEdgeToPivot_Min, m_encoderPositionOffsetRotations);

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempArmSimulation.getIsBroken() && !m_winchSimulation.getIsBroken());
  }

  @Test
  public void NullWinchSimShouldThrowException() {
    assertThrows(IllegalArgumentException.class, () -> {
      ArmSimulation tempArmSimulation = new ArmSimulation(null, m_winchAbsoluteEncoderSim,
          m_armTopRotationsLimit, m_armBottomRotationsLimit, m_armDeltaRotationsBeforeBroken,
          m_grabberBreaksIfOpenBelowThisLimit, m_armHeightFromWinchToPivotPoint,
          m_armLengthFromEdgeToPivot, m_armLengthFromEdgeToPivot_Min,
          m_encoderPositionOffsetRotations);
    });
  }

  @Test
  public void ArmUpWithGrabberInitiallyOpenShouldSucceed() {
    double lengthStringExtended = m_armHeightFromWinchToPivotPoint - 0.5;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimulation tempwinchSimulation = new WinchSimulation(m_winchRelEncoderSim,
        m_winchSpoolDiameterMeters, m_winchTotalStringLenMeters, winchInitialLenSpooled, // this is
                                                                                         // the
                                                                                         // value
                                                                                         // we're
                                                                                         // setting
                                                                                         // to make
                                                                                         // arm
                                                                                         // level
        m_winchInitialStringOrientation, m_winchinvertMotor);

    ArmSimulation tempArmSimulation = new ArmSimulation(tempwinchSimulation,
        m_winchAbsoluteEncoderSim, m_armTopRotationsLimit, m_armBottomRotationsLimit,
        m_armDeltaRotationsBeforeBroken, m_grabberBreaksIfOpenBelowThisLimit,
        m_armHeightFromWinchToPivotPoint, m_armLengthFromEdgeToPivot,
        m_armLengthFromEdgeToPivot_Min, m_encoderPositionOffsetRotations);

    // Set grabber open
    BooleanSupplier isGrabberOpen = () -> true;
    tempArmSimulation.setGrabberOpenSupplier(isGrabberOpen);

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempwinchSimulation.getIsBroken());

    assertTrue(!tempArmSimulation.getIsBroken());
    assertEquals(m_winchAbsoluteEncoder.get() * 360, 90, UnitConversions.kAngleTolerance);
  }

  @Test
  public void ArmDownWithGrabberInitiallyOpenShouldFail() {
    double lengthStringExtended = m_armHeightFromWinchToPivotPoint + 0.5;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimulation tempwinchSimulation = new WinchSimulation(m_winchRelEncoderSim,
        m_winchSpoolDiameterMeters, m_winchTotalStringLenMeters, winchInitialLenSpooled, // this is
                                                                                         // the
                                                                                         // value
                                                                                         // we're
                                                                                         // setting
                                                                                         // to make
                                                                                         // arm
                                                                                         // level
        m_winchInitialStringOrientation, m_winchinvertMotor);

    ArmSimulation tempArmSimulation = new ArmSimulation(tempwinchSimulation,
        m_winchAbsoluteEncoderSim, m_armTopRotationsLimit, m_armBottomRotationsLimit,
        m_armDeltaRotationsBeforeBroken, m_grabberBreaksIfOpenBelowThisLimit,
        m_armHeightFromWinchToPivotPoint, m_armLengthFromEdgeToPivot,
        m_armLengthFromEdgeToPivot_Min, m_encoderPositionOffsetRotations);

    // Set grabber open
    BooleanSupplier isGrabberOpen = () -> true;
    tempArmSimulation.setGrabberOpenSupplier(isGrabberOpen);

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(!tempArmSimulation.getIsBroken());

    // Now that grabber is set open, need to simulate one cycle
    tempArmSimulation.simulationPeriodic();

    assertTrue(tempArmSimulation.getIsBroken());
    assertEquals(m_winchAbsoluteEncoder.get() * 360, 360 - 90, UnitConversions.kAngleTolerance);
  }

  @Test
  public void MovingArmDownwardPastBreakLimitWithGrabberOpenShouldNotMoveArm() {
    double breakLimitSignedDegrees = (m_grabberBreaksIfOpenBelowThisLimit * 360) - 360;
    double initialPosSignedDegrees = breakLimitSignedDegrees + 4;
    double targetPosSignedDegrees = breakLimitSignedDegrees - 4;

    double backArmAbovePivot = -1
        * (m_armLengthFromEdgeToPivot * Math.sin(initialPosSignedDegrees * Math.PI / 180));
    double lengthStringExtended = m_armHeightFromWinchToPivotPoint + backArmAbovePivot;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimulation tempwinchSimulation = new WinchSimulation(m_winchRelEncoderSim,
        m_winchSpoolDiameterMeters, m_winchTotalStringLenMeters, winchInitialLenSpooled, // this is
                                                                                         // the
                                                                                         // value
                                                                                         // we're
                                                                                         // setting
                                                                                         // to make
                                                                                         // arm
                                                                                         // level
        m_winchInitialStringOrientation, m_winchinvertMotor);

    ArmSimulation tempArmSimulation = new ArmSimulation(tempwinchSimulation,
        m_winchAbsoluteEncoderSim, m_armTopRotationsLimit, m_armBottomRotationsLimit,
        m_armDeltaRotationsBeforeBroken, m_grabberBreaksIfOpenBelowThisLimit,
        m_armHeightFromWinchToPivotPoint, m_armLengthFromEdgeToPivot,
        m_armLengthFromEdgeToPivot_Min, m_encoderPositionOffsetRotations);

    // Set grabber open
    BooleanSupplier isGrabberOpen = () -> true;
    tempArmSimulation.setGrabberOpenSupplier(isGrabberOpen);

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(!tempArmSimulation.getIsBroken());

    // Now that grabber is set open, need to simulate one cycle
    tempArmSimulation.simulationPeriodic();

    assertTrue(!tempArmSimulation.getIsBroken());
    double expect = initialPosSignedDegrees + 360;
    double actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);

    // Now calculate how much to turn the winch motor to get it to the target position
    backArmAbovePivot = -1
        * (m_armLengthFromEdgeToPivot * Math.sin(targetPosSignedDegrees * Math.PI / 180));
    lengthStringExtended = m_armHeightFromWinchToPivotPoint + backArmAbovePivot;
    double winchTargetLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    double currentWinchRotations = m_winchRelEncoderSim.getPosition();
    double spoolCircumferenceMeters = Math.PI * m_winchSpoolDiameterMeters;
    double deltaWinchRotations = (winchInitialLenSpooled - winchTargetLenSpooled)
        / spoolCircumferenceMeters;

    m_winchRelEncoderSim.setPosition(currentWinchRotations + deltaWinchRotations);

    // Simulate one cycle for winch, so that it updates
    tempwinchSimulation.simulationPeriodic();
    tempArmSimulation.simulationPeriodic();

    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(!tempArmSimulation.getIsBroken());

    // We expect that the arm gets stuck at the break limit, instead of going all the way to the
    // target degrees
    expect = breakLimitSignedDegrees + 360;
    actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);
  }

  @Test
  public void MovingArmWithGrabberOpenShouldSucceedIfArmIsTowardsTop() {
    double breakLimitSignedDegrees = (m_grabberBreaksIfOpenBelowThisLimit * 360) - 360;
    double initialPosSignedDegrees = breakLimitSignedDegrees + 4;
    double targetPosSignedDegrees = breakLimitSignedDegrees + 8;

    double backArmAbovePivot = -1
        * (m_armLengthFromEdgeToPivot * Math.sin(initialPosSignedDegrees * Math.PI / 180));
    double lengthStringExtended = m_armHeightFromWinchToPivotPoint + backArmAbovePivot;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimulation tempwinchSimulation = new WinchSimulation(m_winchRelEncoderSim,
        m_winchSpoolDiameterMeters, m_winchTotalStringLenMeters, winchInitialLenSpooled, // this is
                                                                                         // the
                                                                                         // value
                                                                                         // we're
                                                                                         // setting
                                                                                         // to make
                                                                                         // arm
                                                                                         // level
        m_winchInitialStringOrientation, m_winchinvertMotor);

    ArmSimulation tempArmSimulation = new ArmSimulation(tempwinchSimulation,
        m_winchAbsoluteEncoderSim, m_armTopRotationsLimit, m_armBottomRotationsLimit,
        m_armDeltaRotationsBeforeBroken, m_grabberBreaksIfOpenBelowThisLimit,
        m_armHeightFromWinchToPivotPoint, m_armLengthFromEdgeToPivot,
        m_armLengthFromEdgeToPivot_Min, m_encoderPositionOffsetRotations);

    // Set grabber open
    BooleanSupplier isGrabberOpen = () -> true;
    tempArmSimulation.setGrabberOpenSupplier(isGrabberOpen);

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(!tempArmSimulation.getIsBroken());

    // Now that grabber is set open, need to simulate one cycle
    tempArmSimulation.simulationPeriodic();

    assertTrue(!tempArmSimulation.getIsBroken());
    double expect = initialPosSignedDegrees + 360;
    double actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);

    // Now calculate how much to turn the winch motor to get it to the target position
    backArmAbovePivot = -1
        * (m_armLengthFromEdgeToPivot * Math.sin(targetPosSignedDegrees * Math.PI / 180));
    lengthStringExtended = m_armHeightFromWinchToPivotPoint + backArmAbovePivot;
    double winchTargetLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    double currentWinchRotations = m_winchRelEncoderSim.getPosition();
    double spoolCircumferenceMeters = Math.PI * m_winchSpoolDiameterMeters;
    double deltaWinchRotations = (winchInitialLenSpooled - winchTargetLenSpooled)
        / spoolCircumferenceMeters;

    m_winchRelEncoderSim.setPosition(currentWinchRotations + deltaWinchRotations);

    // Simulate one cycle for winch, so that it updates
    tempwinchSimulation.simulationPeriodic();
    tempArmSimulation.simulationPeriodic();

    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(!tempArmSimulation.getIsBroken());

    // We expect that the arm gets stuck at the break limit, instead of going all the way to the
    // target degrees
    expect = targetPosSignedDegrees + 360;
    actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);
  }

  @Test
  public void MovingArmUpwardFromBreakLimitWithGrabberOpenShouldSucceed() {
    double breakLimitSignedDegrees = (m_grabberBreaksIfOpenBelowThisLimit * 360) - 360;
    double initialPosSignedDegrees = breakLimitSignedDegrees;
    double targetPosSignedDegrees = breakLimitSignedDegrees + 8;

    double backArmAbovePivot = -1
        * (m_armLengthFromEdgeToPivot * Math.sin(initialPosSignedDegrees * Math.PI / 180));
    double lengthStringExtended = m_armHeightFromWinchToPivotPoint + backArmAbovePivot;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimulation tempwinchSimulation = new WinchSimulation(m_winchRelEncoderSim,
        m_winchSpoolDiameterMeters, m_winchTotalStringLenMeters, winchInitialLenSpooled, // this is
                                                                                         // the
                                                                                         // value
                                                                                         // we're
                                                                                         // setting
                                                                                         // to make
                                                                                         // arm
                                                                                         // level
        m_winchInitialStringOrientation, m_winchinvertMotor);

    ArmSimulation tempArmSimulation = new ArmSimulation(tempwinchSimulation,
        m_winchAbsoluteEncoderSim, m_armTopRotationsLimit, m_armBottomRotationsLimit,
        m_armDeltaRotationsBeforeBroken, m_grabberBreaksIfOpenBelowThisLimit,
        m_armHeightFromWinchToPivotPoint, m_armLengthFromEdgeToPivot,
        m_armLengthFromEdgeToPivot_Min, m_encoderPositionOffsetRotations);

    // Set grabber open
    BooleanSupplier isGrabberOpen = () -> true;
    tempArmSimulation.setGrabberOpenSupplier(isGrabberOpen);

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(!tempArmSimulation.getIsBroken());

    // Now that grabber is set open, need to simulate one cycle
    tempArmSimulation.simulationPeriodic();

    assertTrue(!tempArmSimulation.getIsBroken());
    double expect = initialPosSignedDegrees + 360;
    double actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);

    // Now calculate how much to turn the winch motor to get it to the target position
    backArmAbovePivot = -1
        * (m_armLengthFromEdgeToPivot * Math.sin(targetPosSignedDegrees * Math.PI / 180));
    lengthStringExtended = m_armHeightFromWinchToPivotPoint + backArmAbovePivot;
    double winchTargetLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    double currentWinchRotations = m_winchRelEncoderSim.getPosition();
    double spoolCircumferenceMeters = Math.PI * m_winchSpoolDiameterMeters;
    double deltaWinchRotations = (winchInitialLenSpooled - winchTargetLenSpooled)
        / spoolCircumferenceMeters;

    m_winchRelEncoderSim.setPosition(currentWinchRotations + deltaWinchRotations);

    // Simulate one cycle for winch, so that it updates
    tempwinchSimulation.simulationPeriodic();
    tempArmSimulation.simulationPeriodic();

    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(!tempArmSimulation.getIsBroken());

    // We expect that the arm gets stuck at the break limit, instead of going all the way to the
    // target degrees
    expect = targetPosSignedDegrees + 360;
    actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);
  }

  @Test
  public void MovingAlreadyBrokenArmShouldNotMoveArm() {
    double breakLimitSignedDegrees = (m_grabberBreaksIfOpenBelowThisLimit * 360) - 360;
    double initialPosSignedDegrees = breakLimitSignedDegrees - 4;
    double targetPosSignedDegrees = breakLimitSignedDegrees + 8;

    double backArmAbovePivot = -1
        * (m_armLengthFromEdgeToPivot * Math.sin(initialPosSignedDegrees * Math.PI / 180));
    double lengthStringExtended = m_armHeightFromWinchToPivotPoint + backArmAbovePivot;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimulation tempwinchSimulation = new WinchSimulation(m_winchRelEncoderSim,
        m_winchSpoolDiameterMeters, m_winchTotalStringLenMeters, winchInitialLenSpooled, // this is
                                                                                         // the
                                                                                         // value
                                                                                         // we're
                                                                                         // setting
                                                                                         // to make
                                                                                         // arm
                                                                                         // level
        m_winchInitialStringOrientation, m_winchinvertMotor);

    ArmSimulation tempArmSimulation = new ArmSimulation(tempwinchSimulation,
        m_winchAbsoluteEncoderSim, m_armTopRotationsLimit, m_armBottomRotationsLimit,
        m_armDeltaRotationsBeforeBroken, m_grabberBreaksIfOpenBelowThisLimit,
        m_armHeightFromWinchToPivotPoint, m_armLengthFromEdgeToPivot,
        m_armLengthFromEdgeToPivot_Min, m_encoderPositionOffsetRotations);

    // Set grabber open
    BooleanSupplier isGrabberOpen = () -> true;
    tempArmSimulation.setGrabberOpenSupplier(isGrabberOpen);

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(!tempArmSimulation.getIsBroken());

    // Now that grabber is set open, need to simulate one cycle
    tempArmSimulation.simulationPeriodic();

    assertTrue(tempArmSimulation.getIsBroken());
    double expect = initialPosSignedDegrees + 360;
    double actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);

    // Now calculate how much to turn the winch motor to get it to the target position
    backArmAbovePivot = -1
        * (m_armLengthFromEdgeToPivot * Math.sin(targetPosSignedDegrees * Math.PI / 180));
    lengthStringExtended = m_armHeightFromWinchToPivotPoint + backArmAbovePivot;
    double winchTargetLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    double currentWinchRotations = m_winchRelEncoderSim.getPosition();
    double spoolCircumferenceMeters = Math.PI * m_winchSpoolDiameterMeters;
    double deltaWinchRotations = (winchInitialLenSpooled - winchTargetLenSpooled)
        / spoolCircumferenceMeters;

    m_winchRelEncoderSim.setPosition(currentWinchRotations + deltaWinchRotations);

    // Simulate one cycle for winch, so that it updates
    tempwinchSimulation.simulationPeriodic();
    tempArmSimulation.simulationPeriodic();

    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(tempArmSimulation.getIsBroken());

    // We expect that the arm gets stuck at the break limit, instead of going all the way to the
    // target degrees
    expect = initialPosSignedDegrees + 360;
    actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);
  }

  private void CreateWithNDegreeArm_Helper(double backArmAbovePivot,
      double expectedDegrees,
      boolean expectArmBroken) {
    double lengthStringExtended = m_armHeightFromWinchToPivotPoint + backArmAbovePivot;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimulation tempwinchSimulation = new WinchSimulation(m_winchRelEncoderSim,
        m_winchSpoolDiameterMeters, m_winchTotalStringLenMeters, winchInitialLenSpooled, // this is
                                                                                         // the
                                                                                         // value
                                                                                         // we're
                                                                                         // setting
                                                                                         // to make
                                                                                         // arm
                                                                                         // level
        m_winchInitialStringOrientation, m_winchinvertMotor);

    ArmSimulation tempArmSimulation = new ArmSimulation(tempwinchSimulation,
        m_winchAbsoluteEncoderSim, m_armTopRotationsLimit, m_armBottomRotationsLimit,
        m_armDeltaRotationsBeforeBroken, m_grabberBreaksIfOpenBelowThisLimit,
        m_armHeightFromWinchToPivotPoint, m_armLengthFromEdgeToPivot,
        m_armLengthFromEdgeToPivot_Min, m_encoderPositionOffsetRotations);

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempwinchSimulation.getIsBroken());

    if (expectArmBroken) {
      assertTrue(tempArmSimulation.getIsBroken());
    }
    else {
      assertTrue(!tempArmSimulation.getIsBroken());
      assertEquals(m_winchAbsoluteEncoder.get() * 360,
          expectedDegrees,
          UnitConversions.kAngleTolerance);
    }
  }

  @Test
  public void CreateWithLevelArmShouldSucceed() {
    CreateWithNDegreeArm_Helper(0, 0, false);
  }

  @Test
  public void CreateWith45DegreeArmShouldSucceed() {
    CreateWithNDegreeArm_Helper(-0.35355, 45, false);
  }

  @Test
  public void CreateWith30DegreeArmShouldSucceed() {
    CreateWithNDegreeArm_Helper(-0.25, 30, false);
  }

  @Test
  public void CreateWith90DegreeArmShouldSucceed() {
    CreateWithNDegreeArm_Helper(-0.5, 90, false);
  }

  @Test
  public void CreateWithNegative90DegreeArmShouldSucceed() {
    CreateWithNDegreeArm_Helper(0.5, 360 - 90, false);
  }

  @Test
  public void CreateWith91DegreeArmShouldFail() {
    double amountBeyondLimit = 0.0001;

    CreateWithNDegreeArm_Helper(-0.5 - amountBeyondLimit, 90, true);
  }

  @Test
  public void CreateWithNegative91DegreeArmShouldNotBreakArm() {
    double amountBeyondLimit = 0.0001;

    CreateWithNDegreeArm_Helper(0.5 + amountBeyondLimit, 360 - 90, false);
  }

  @Test
  public void CreateWith98DegreeArmShouldFail() {
    double amountBeyondLimit = 0.1;

    CreateWithNDegreeArm_Helper(-0.5 - amountBeyondLimit, 90, true);
  }

  @Test
  public void CreateWithNegative98DegreeArmShouldNotBreakArm() {
    double amountBeyondLimit = 0.1;

    CreateWithNDegreeArm_Helper(0.5 + amountBeyondLimit, 360 - 90, false);
  }

  @Test
  public void CreateWithNegative45DegreeArmShouldSucceed() {
    CreateWithNDegreeArm_Helper(0.35355, 360 - 45, false);
  }

  // Sometimes, the absolute encoder is offset, and 0 isn't level
  @Test
  public void CreateWithOffsetShouldSucceed() {
    double lengthStringExtended = m_armHeightFromWinchToPivotPoint - 0.35355;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;
    double expectedDegrees = 45 + 90;
    double offsetRotations = 0.25;

    WinchSimulation tempwinchSimulation = new WinchSimulation(m_winchRelEncoderSim,
        m_winchSpoolDiameterMeters, m_winchTotalStringLenMeters, winchInitialLenSpooled, // this is
                                                                                         // the
                                                                                         // value
                                                                                         // we're
                                                                                         // setting
                                                                                         // to make
                                                                                         // arm
                                                                                         // level
        m_winchInitialStringOrientation, m_winchinvertMotor);

    ArmSimulation tempArmSimulation = new ArmSimulation(tempwinchSimulation,
        m_winchAbsoluteEncoderSim, m_armTopRotationsLimit + offsetRotations,
        m_armBottomRotationsLimit + offsetRotations, m_armDeltaRotationsBeforeBroken,
        m_grabberBreaksIfOpenBelowThisLimit + offsetRotations, m_armHeightFromWinchToPivotPoint,
        m_armLengthFromEdgeToPivot, m_armLengthFromEdgeToPivot_Min, offsetRotations);

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(!tempArmSimulation.getIsBroken());
    assertEquals(m_winchAbsoluteEncoder.get() * 360,
        expectedDegrees,
        UnitConversions.kAngleTolerance);
  }

  @Test
  public void Offset0ArmRotationBy180DegreesShouldWork() {
    double position = 0;
    double offset = 0.5; // 180
    double expectedResult = 0.5;

    double actualResult = ArmSimulation.offsetArmRotationPosition(position, offset);
    assertEquals(expectedResult, actualResult);
  }

  @Test
  public void Offset90ArmRotationBy180DegreesShouldWork() {
    double position = 0.25;
    double offset = 0.5; // 180
    double expectedResult = 0.75;

    double actualResult = ArmSimulation.offsetArmRotationPosition(position, offset);
    assertEquals(expectedResult, actualResult);
  }

  @Test
  public void Offset216ArmRotationBy180DegreesShouldWork() {
    double position = 0.6;
    double offset = 0.5; // 180
    double expectedResult = 0.1;

    double actualResult = ArmSimulation.offsetArmRotationPosition(position, offset);
    assertEquals(expectedResult, actualResult, UnitConversions.kAngleTolerance);
  }

  @Test
  public void Offset180ArmRotationByNegative90DegreesShouldWork() {
    double position = 0.5;
    double offset = -0.25; // 90
    double expectedResult = 0.25;

    double actualResult = ArmSimulation.offsetArmRotationPosition(position, offset);
    assertEquals(expectedResult, actualResult);
  }
}
