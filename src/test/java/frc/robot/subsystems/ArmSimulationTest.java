package frc.robot.Simulation;

import frc.robot.Subsystems.RelativeEncoderSim;
import frc.robot.Simulation.WinchSimulation.StringOrientation;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

public class ArmSimulationTest {
    private final double m_armTopAngleRotations = 0.78;
    private final double m_armBottomAngleRotations = 0.56;
    private final double m_armDeltaRotationsBeforeBroken = 0.02;
    private final double m_winchSpoolDiameterMeters = 0.01; // (1 centimeter)
    private final double m_winchTotalStringLenMeters = 5;
    private final double m_winchInitialLenSpooled = 1;
    private final StringOrientation m_winchInitialStringOrientation = StringOrientation.BackOfRobot;
    private final boolean m_winchinvertMotor = false;

    private WinchSimulation m_winchSimulation;
    private RelativeEncoderSim m_winchRelEncoderSim;

    @BeforeEach
    public void setUp() {
      m_winchRelEncoderSim = new RelativeEncoderSim(null, true); // test-mode

      m_winchSimulation = new WinchSimulation(
        m_winchRelEncoderSim,
        m_winchSpoolDiameterMeters,
        m_winchTotalStringLenMeters,
        m_winchInitialLenSpooled,
        m_winchInitialStringOrientation,
        m_winchinvertMotor);
    }

    @Test
    public void CreateArmSimulationShouldSucceed() {
      ArmSimulation tempArmSimulation = new ArmSimulation(
        m_winchSimulation,
        m_armTopAngleRotations,
        m_armBottomAngleRotations,
        m_armDeltaRotationsBeforeBroken);

      assertTrue(tempArmSimulation != null);
    }
}

