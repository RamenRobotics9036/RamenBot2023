package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.Constants;
import frc.robot.simulation.winch.WinchSimModel;
import frc.robot.simulation.winch.WinchSimModel.WindingOrientation;
import frc.robot.subsystems.DutyCycleEncoderSim2;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests the ArmSimulation class.
 */
public class ArmSimulationTest {
  private final ArmSimulationParams m_defaultArmParams;
  private final double m_winchSpoolDiameterMeters = 0.01; // (1 centimeter)
  private final double m_winchTotalStringLenMeters = 5;
  private final double m_winchInitialLenSpooled = 4;
  private final WindingOrientation m_winchInitialStringOrientation = WindingOrientation.BackOfRobot;
  private final boolean m_winchinvertMotor = false;

  private WinchSimModel m_winchSimulation;
  private DutyCycleEncoder m_winchAbsoluteEncoder = null;
  private DutyCycleEncoderSim m_winchAbsoluteEncoderSim = null;

  /**
   * Constructor.
   */
  public ArmSimulationTest() {
    m_defaultArmParams = new ArmSimulationParams(0.25, // topRotationsLimit
        0.75, // bottomRotationsLimit
        0, // deltaRotationsBeforeBroken
        0.80, // grabberBreaksIfOpenBelowThisLimit
        1, // heightFromWinchToPivotPoint
        0.5, // armLengthFromEdgeToPivot
        0.1, // armLengthFromEdgeToPivotMin
        0); // encoderRotationsOffset
  }

  /**
   * Runs before every test.
   */
  @BeforeEach
  public void setUp() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    m_winchSimulation = new WinchSimModel(m_winchSpoolDiameterMeters, m_winchTotalStringLenMeters,
        m_winchInitialLenSpooled, m_winchInitialStringOrientation, m_winchinvertMotor);

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

  private ArmSimulation createDefaultArmHelper(WinchSimModel winchSimulation,
      boolean initialIsGrabberOpen) {
    // Create a DoubleSupplier that gets the value getStringUnspooledLen()
    DoubleSupplier stringUnspooledLenSupplier = () -> {
      return winchSimulation.getStringUnspooledLen();
    };

    // Copy parameters from the Default Arm Parameters
    ArmSimulation armSimulation = new ArmSimulation(stringUnspooledLenSupplier,
        m_winchAbsoluteEncoderSim, m_defaultArmParams);

    // Set grabber
    BooleanSupplier isGrabberOpen = () -> initialIsGrabberOpen;
    armSimulation.setGrabberOpenSupplier(isGrabberOpen);

    assertTrue(armSimulation != null);
    assertTrue(!winchSimulation.getIsBroken());
    assertTrue(!armSimulation.getIsBroken());

    return armSimulation;
  }

  private ArmSimulation createDefaultArm() {
    return createDefaultArmHelper(m_winchSimulation, false);
  }

  private WinchSimModel createWinchSimulation(double winchInitialLenSpooled) {
    WinchSimModel winchSimulation = new WinchSimModel(m_winchSpoolDiameterMeters,
        m_winchTotalStringLenMeters, winchInitialLenSpooled, m_winchInitialStringOrientation,
        m_winchinvertMotor);

    assertTrue(winchSimulation != null);
    assertTrue(!winchSimulation.getIsBroken());

    return winchSimulation;
  }

  @Test
  public void createArmSimulationShouldSucceed() {
    ArmSimulation tempArmSimulation = createDefaultArm();

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempArmSimulation.getIsBroken() && !m_winchSimulation.getIsBroken());
  }

  @Test
  public void nullWinchSimShouldThrowException() {
    assertThrows(IllegalArgumentException.class, () -> {
      ArmSimulation tempArmSimulation = new ArmSimulation(null, m_winchAbsoluteEncoderSim,
          m_defaultArmParams);
      assertTrue(tempArmSimulation != null);
    });
  }

  @Test
  public void armUpWithGrabberInitiallyOpenShouldSucceed() {
    double lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint - 0.5;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimModel tempwinchSimulation = createWinchSimulation(winchInitialLenSpooled);
    ArmSimulation tempArmSimulation = createDefaultArmHelper(tempwinchSimulation, true);

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(!tempArmSimulation.getIsBroken());
    assertEquals(m_winchAbsoluteEncoder.get() * 360, 90, UnitConversions.kAngleTolerance);
  }

  @Test
  public void armDownWithGrabberInitiallyOpenShouldFail() {
    double lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint + 0.5;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimModel tempwinchSimulation = createWinchSimulation(winchInitialLenSpooled);
    ArmSimulation tempArmSimulation = createDefaultArmHelper(tempwinchSimulation, true);

    // Now that grabber is set open, need to simulate one cycle
    tempArmSimulation.simulationPeriodic();

    assertTrue(tempArmSimulation.getIsBroken());
    assertEquals(m_winchAbsoluteEncoder.get() * 360, 360 - 90, UnitConversions.kAngleTolerance);
  }

  @Test
  public void movingArmDownwardPastBreakLimitWithGrabberOpenShouldNotMoveArm() {
    double breakLimitSignedDegrees = (m_defaultArmParams.m_grabberBreaksIfOpenBelowThisLimit * 360)
        - 360;
    double initialPosSignedDegrees = breakLimitSignedDegrees + 4;

    double backArmAbovePivot = -1 * (m_defaultArmParams.m_armLengthFromEdgeToPivot
        * Math.sin(initialPosSignedDegrees * Math.PI / 180));
    double lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint
        + backArmAbovePivot;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimModel tempwinchSimulation = createWinchSimulation(winchInitialLenSpooled);
    ArmSimulation tempArmSimulation = createDefaultArmHelper(tempwinchSimulation, true);

    // Initialize the number of rotations
    tempwinchSimulation.updateNewLenSpooled(0);

    // Now that grabber is set open, need to simulate one cycle
    tempArmSimulation.simulationPeriodic();

    assertTrue(!tempArmSimulation.getIsBroken());
    double expect = initialPosSignedDegrees + 360;
    double actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);

    double targetPosSignedDegrees = breakLimitSignedDegrees - 4;

    // Now calculate how much to turn the winch motor to get it to the target position
    backArmAbovePivot = -1 * (m_defaultArmParams.m_armLengthFromEdgeToPivot
        * Math.sin(targetPosSignedDegrees * Math.PI / 180));
    lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint + backArmAbovePivot;
    double winchTargetLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    double spoolCircumferenceMeters = Math.PI * m_winchSpoolDiameterMeters;
    double deltaWinchRotations = (winchInitialLenSpooled - winchTargetLenSpooled)
        / spoolCircumferenceMeters;

    // Simulate one cycle for winch, so that it updates
    tempwinchSimulation.updateNewLenSpooled(deltaWinchRotations);
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
  public void movingArmWithGrabberOpenShouldSucceedIfArmIsTowardsTop() {
    double breakLimitSignedDegrees = (m_defaultArmParams.m_grabberBreaksIfOpenBelowThisLimit * 360)
        - 360;
    double initialPosSignedDegrees = breakLimitSignedDegrees + 4;

    double backArmAbovePivot = -1 * (m_defaultArmParams.m_armLengthFromEdgeToPivot
        * Math.sin(initialPosSignedDegrees * Math.PI / 180));
    double lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint
        + backArmAbovePivot;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimModel tempwinchSimulation = createWinchSimulation(winchInitialLenSpooled);
    ArmSimulation tempArmSimulation = createDefaultArmHelper(tempwinchSimulation, true);

    // Initialize the number of rotations
    tempwinchSimulation.updateNewLenSpooled(0);

    // Now that grabber is set open, need to simulate one cycle
    tempArmSimulation.simulationPeriodic();

    assertTrue(!tempArmSimulation.getIsBroken());
    double expect = initialPosSignedDegrees + 360;
    double actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);

    double targetPosSignedDegrees = breakLimitSignedDegrees + 8;

    // Now calculate how much to turn the winch motor to get it to the target position
    backArmAbovePivot = -1 * (m_defaultArmParams.m_armLengthFromEdgeToPivot
        * Math.sin(targetPosSignedDegrees * Math.PI / 180));
    lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint + backArmAbovePivot;
    double winchTargetLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    double spoolCircumferenceMeters = Math.PI * m_winchSpoolDiameterMeters;
    double deltaWinchRotations = (winchInitialLenSpooled - winchTargetLenSpooled)
        / spoolCircumferenceMeters;

    // Simulate one cycle for winch, so that it updates
    tempwinchSimulation.updateNewLenSpooled(deltaWinchRotations);
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
  public void movingArmUpwardFromBreakLimitWithGrabberOpenShouldSucceed() {
    double breakLimitSignedDegrees = (m_defaultArmParams.m_grabberBreaksIfOpenBelowThisLimit * 360)
        - 360;
    double initialPosSignedDegrees = breakLimitSignedDegrees;

    double backArmAbovePivot = -1 * (m_defaultArmParams.m_armLengthFromEdgeToPivot
        * Math.sin(initialPosSignedDegrees * Math.PI / 180));
    double lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint
        + backArmAbovePivot;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimModel tempwinchSimulation = createWinchSimulation(winchInitialLenSpooled);
    ArmSimulation tempArmSimulation = createDefaultArmHelper(tempwinchSimulation, true);

    // Initialize the number of rotations
    tempwinchSimulation.updateNewLenSpooled(0);

    // Now that grabber is set open, need to simulate one cycle
    tempArmSimulation.simulationPeriodic();

    assertTrue(!tempArmSimulation.getIsBroken());
    double expect = initialPosSignedDegrees + 360;
    double actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);

    double targetPosSignedDegrees = breakLimitSignedDegrees + 8;

    // Now calculate how much to turn the winch motor to get it to the target position
    backArmAbovePivot = -1 * (m_defaultArmParams.m_armLengthFromEdgeToPivot
        * Math.sin(targetPosSignedDegrees * Math.PI / 180));
    lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint + backArmAbovePivot;
    double winchTargetLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    double spoolCircumferenceMeters = Math.PI * m_winchSpoolDiameterMeters;
    double deltaWinchRotations = (winchInitialLenSpooled - winchTargetLenSpooled)
        / spoolCircumferenceMeters;

    // Simulate one cycle for winch, so that it updates
    tempwinchSimulation.updateNewLenSpooled(deltaWinchRotations);
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
  public void movingAlreadyBrokenArmShouldNotMoveArm() {
    double breakLimitSignedDegrees = (m_defaultArmParams.m_grabberBreaksIfOpenBelowThisLimit * 360)
        - 360;
    double initialPosSignedDegrees = breakLimitSignedDegrees - 4;

    double backArmAbovePivot = -1 * (m_defaultArmParams.m_armLengthFromEdgeToPivot
        * Math.sin(initialPosSignedDegrees * Math.PI / 180));
    double lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint
        + backArmAbovePivot;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    WinchSimModel tempwinchSimulation = createWinchSimulation(winchInitialLenSpooled);
    ArmSimulation tempArmSimulation = createDefaultArmHelper(tempwinchSimulation, true);

    // Now that grabber is set open, need to simulate one cycle
    tempArmSimulation.simulationPeriodic();

    assertTrue(tempArmSimulation.getIsBroken());
    double expect = initialPosSignedDegrees + 360;
    double actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);

    double targetPosSignedDegrees = breakLimitSignedDegrees + 8;

    // Now calculate how much to turn the winch motor to get it to the target position
    backArmAbovePivot = -1 * (m_defaultArmParams.m_armLengthFromEdgeToPivot
        * Math.sin(targetPosSignedDegrees * Math.PI / 180));
    lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint + backArmAbovePivot;
    double winchTargetLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    double spoolCircumferenceMeters = Math.PI * m_winchSpoolDiameterMeters;
    double deltaWinchRotations = (winchInitialLenSpooled - winchTargetLenSpooled)
        / spoolCircumferenceMeters;

    // Simulate one cycle for winch, so that it updates
    tempwinchSimulation.updateNewLenSpooled(deltaWinchRotations);
    tempArmSimulation.simulationPeriodic();

    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(tempArmSimulation.getIsBroken());

    // We expect that the arm gets stuck at the break limit, instead of going all the way to the
    // target degrees
    expect = initialPosSignedDegrees + 360;
    actual = m_winchAbsoluteEncoder.get() * 360;
    assertEquals(expect, actual, UnitConversions.kAngleTolerance);
  }

  private void createWithDegreeArmHelper(double backArmAbovePivot,
      double expectedDegrees,
      boolean expectArmBroken) {
    double lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint
        + backArmAbovePivot;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;

    // $TODO - Can I call createDefaultArmHelper here?
    WinchSimModel tempwinchSimulation = createWinchSimulation(winchInitialLenSpooled);

    // Create a DoubleSupplier that gets the value getStringUnspooledLen()
    DoubleSupplier stringUnspooledLenSupplier = () -> {
      return tempwinchSimulation.getStringUnspooledLen();
    };

    ArmSimulation tempArmSimulation = new ArmSimulation(stringUnspooledLenSupplier,
        m_winchAbsoluteEncoderSim, m_defaultArmParams);

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
  public void createWithLevelArmShouldSucceed() {
    createWithDegreeArmHelper(0, 0, false);
  }

  @Test
  public void createWith45DegreeArmShouldSucceed() {
    createWithDegreeArmHelper(-0.35355, 45, false);
  }

  @Test
  public void createWith30DegreeArmShouldSucceed() {
    createWithDegreeArmHelper(-0.25, 30, false);
  }

  @Test
  public void createWith90DegreeArmShouldSucceed() {
    createWithDegreeArmHelper(-0.5, 90, false);
  }

  @Test
  public void createWithNegative90DegreeArmShouldSucceed() {
    createWithDegreeArmHelper(0.5, 360 - 90, false);
  }

  @Test
  public void createWith91DegreeArmShouldFail() {
    double amountBeyondLimit = 0.0001;

    createWithDegreeArmHelper(-0.5 - amountBeyondLimit, 90, true);
  }

  @Test
  public void createWithNegative91DegreeArmShouldNotBreakArm() {
    double amountBeyondLimit = 0.0001;

    createWithDegreeArmHelper(0.5 + amountBeyondLimit, 360 - 90, false);
  }

  @Test
  public void createWith98DegreeArmShouldFail() {
    double amountBeyondLimit = 0.1;

    createWithDegreeArmHelper(-0.5 - amountBeyondLimit, 90, true);
  }

  @Test
  public void createWithNegative98DegreeArmShouldNotBreakArm() {
    double amountBeyondLimit = 0.1;

    createWithDegreeArmHelper(0.5 + amountBeyondLimit, 360 - 90, false);
  }

  @Test
  public void createWithNegative45DegreeArmShouldSucceed() {
    createWithDegreeArmHelper(0.35355, 360 - 45, false);
  }

  // Sometimes, the absolute encoder is offset, and 0 isn't level
  @Test
  public void createWithOffsetShouldSucceed() {
    double lengthStringExtended = m_defaultArmParams.m_heightFromWinchToPivotPoint - 0.35355;
    double winchInitialLenSpooled = m_winchTotalStringLenMeters - lengthStringExtended;
    double offsetRotations = 0.25;

    WinchSimModel tempwinchSimulation = createWinchSimulation(winchInitialLenSpooled);

    // Create a DoubleSupplier that gets the value getStringUnspooledLen()
    DoubleSupplier stringUnspooledLenSupplier = () -> {
      return tempwinchSimulation.getStringUnspooledLen();
    };

    ArmSimulationParamsBuilder tempArmParamsBuilder = new ArmSimulationParamsBuilder(
        m_defaultArmParams);

    tempArmParamsBuilder
        .setTopRotationsLimit(m_defaultArmParams.m_topRotationsLimit + offsetRotations)
        .setBottomRotationsLimit(m_defaultArmParams.m_bottomRotationsLimit + offsetRotations)
        .setGrabberBreaksIfOpenBelowThisLimit(
            m_defaultArmParams.m_grabberBreaksIfOpenBelowThisLimit + offsetRotations)
        .setEncoderRotationsOffset(offsetRotations);

    ArmSimulation tempArmSimulation = new ArmSimulation(stringUnspooledLenSupplier,
        m_winchAbsoluteEncoderSim, tempArmParamsBuilder.build());

    assertTrue(tempArmSimulation != null);
    assertTrue(!tempwinchSimulation.getIsBroken());
    assertTrue(!tempArmSimulation.getIsBroken());

    double expectedDegrees = 45 + 90;
    assertEquals(m_winchAbsoluteEncoder.get() * 360,
        expectedDegrees,
        UnitConversions.kAngleTolerance);
  }

  @Test
  public void offset0ArmRotationBy180DegreesShouldWork() {
    double position = 0;
    double offset = 0.5; // 180
    double expectedResult = 0.5;

    double actualResult = ArmSimulation.offsetArmRotationPosition(position, offset);
    assertEquals(expectedResult, actualResult);
  }

  @Test
  public void offset90ArmRotationBy180DegreesShouldWork() {
    double position = 0.25;
    double offset = 0.5; // 180
    double expectedResult = 0.75;

    double actualResult = ArmSimulation.offsetArmRotationPosition(position, offset);
    assertEquals(expectedResult, actualResult);
  }

  @Test
  public void offset216ArmRotationBy180DegreesShouldWork() {
    double position = 0.6;
    double offset = 0.5; // 180
    double expectedResult = 0.1;

    double actualResult = ArmSimulation.offsetArmRotationPosition(position, offset);
    assertEquals(expectedResult, actualResult, UnitConversions.kAngleTolerance);
  }

  @Test
  public void offset180ArmRotationByNegative90DegreesShouldWork() {
    double position = 0.5;
    double offset = -0.25; // 90
    double expectedResult = 0.25;

    double actualResult = ArmSimulation.offsetArmRotationPosition(position, offset);
    assertEquals(expectedResult, actualResult);
  }
}
