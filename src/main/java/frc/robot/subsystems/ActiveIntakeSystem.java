package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ActiveIntakeSystem extends SubsystemBase {
  private CANSparkMax m_activeIntakeMotor1;
  private CANSparkMax m_activeIntakeMotor2;
  private XboxController m_controller;

  public ActiveIntakeSystem(CANSparkMax m_activeIntakeMotor1, CANSparkMax m_activeIntakeMotor2, XboxController m_controller) {
    this.m_activeIntakeMotor1 = m_activeIntakeMotor1;
    this.m_activeIntakeMotor2 = m_activeIntakeMotor2;
    this.m_controller = m_controller;

    m_activeIntakeMotor1.setSmartCurrentLimit(5);
    m_activeIntakeMotor1.setSmartCurrentLimit(5);
  }

  @Override
  public void periodic() {
    if (m_controller.getAButton()) {
      m_activeIntakeMotor1.set(0.2);
      m_activeIntakeMotor2.set(0.2);
    } else {
      m_activeIntakeMotor1.set(-0.2);
      m_activeIntakeMotor2.set(-0.2);
    }
  }
}
