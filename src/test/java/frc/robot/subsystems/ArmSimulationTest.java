package frc.robot.Simulation;

import edu.wpi.first.hal.HAL;
import frc.robot.Subsystems.RelativeEncoderSim;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.Simulation.WinchSimulation.StringOrientation;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import frc.robot.Constants;

public class ArmSimulationTest {
    private final double m_armTopRotationsLimit = 0.78;
    private final double m_armBottomRotationsLimit = 0.56;
    private final double m_armDeltaRotationsBeforeBroken = 0.02;
    private final double m_grabberBreaksIfOpenBelowThisLimit = 0.60;
    private final double m_winchSpoolDiameterMeters = 0.01; // (1 centimeter)
    private final double m_winchTotalStringLenMeters = 5;
    private final double m_winchInitialLenSpooled = 1;
    private final StringOrientation m_winchInitialStringOrientation = StringOrientation.BackOfRobot;
    private final boolean m_winchinvertMotor = false;

    private WinchSimulation m_winchSimulation;
    private RelativeEncoderSim m_winchRelEncoderSim;
    private DutyCycleEncoder m_winchAbsoluteEncoder = null;
    private DutyCycleEncoderSim m_winchAbsoluteEncoderSim = null;

    @BeforeEach
    public void setUp() {
      assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

      m_winchRelEncoderSim = new RelativeEncoderSim(null, true); // test-mode

      m_winchSimulation = new WinchSimulation(
        m_winchRelEncoderSim,
        m_winchSpoolDiameterMeters,
        m_winchTotalStringLenMeters,
        m_winchInitialLenSpooled,
        m_winchInitialStringOrientation,
        m_winchinvertMotor);

      m_winchAbsoluteEncoder = new DutyCycleEncoder(Constants.OperatorConstants.kAbsoluteEncoderWinchChannel);
      m_winchAbsoluteEncoderSim = new DutyCycleEncoderSim(m_winchAbsoluteEncoder);
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
      ArmSimulation tempArmSimulation = new ArmSimulation(
        m_winchSimulation,
        m_winchAbsoluteEncoderSim,
        m_armTopRotationsLimit,
        m_armBottomRotationsLimit,
        m_armDeltaRotationsBeforeBroken,
        m_grabberBreaksIfOpenBelowThisLimit);

      assertTrue(tempArmSimulation != null);
    }

    @Test
    public void NullWinchSimShouldThrowException() {
      assertThrows(IllegalArgumentException.class, () -> {
        ArmSimulation tempArmSimulation = new ArmSimulation(
          null,
          m_winchAbsoluteEncoderSim,
          m_armTopRotationsLimit,
          m_armBottomRotationsLimit,
          m_armDeltaRotationsBeforeBroken,
          m_grabberBreaksIfOpenBelowThisLimit);
      });
    }

    @Test
    public void BottomLimitMinusDeltaLessThanZeroShouldThrowException() {
      assertThrows(IllegalArgumentException.class, () -> {
        ArmSimulation tempArmSimulation = new ArmSimulation(
          m_winchSimulation,
          m_winchAbsoluteEncoderSim,
          m_armTopRotationsLimit,
          0.01,
          0.02,
          m_grabberBreaksIfOpenBelowThisLimit);
      });
    }

    @Test
    public void TopLimitPlusDeltaGreaterThanOneShouldThrowException() {
      assertThrows(IllegalArgumentException.class, () -> {
        ArmSimulation tempArmSimulation = new ArmSimulation(
          m_winchSimulation,
          m_winchAbsoluteEncoderSim,
          0.99,
          m_armBottomRotationsLimit,
          0.02,
          m_grabberBreaksIfOpenBelowThisLimit);
      });
    }

    @Test
    public void BottomLimitMinusDeltaEqualToZeroShouldSucceed() {
      ArmSimulation tempArmSimulation = new ArmSimulation(
        m_winchSimulation,
        m_winchAbsoluteEncoderSim,
        m_armTopRotationsLimit,
        0.01,
        0.01,
        m_grabberBreaksIfOpenBelowThisLimit);

      assertTrue(tempArmSimulation != null);
    }

    @Test
    public void TopLimitPlusDeltaEqualToOneShouldSucceed() {
      ArmSimulation tempArmSimulation = new ArmSimulation(
        m_winchSimulation,
        m_winchAbsoluteEncoderSim,
        0.99,
        m_armBottomRotationsLimit,
        0.01,
        m_grabberBreaksIfOpenBelowThisLimit);

      assertTrue(tempArmSimulation != null);
    }
}

