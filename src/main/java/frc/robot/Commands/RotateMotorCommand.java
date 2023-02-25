package frc.robot.Commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateMotorCommand extends CommandBase {
    private double distance;
    private double gearBoxRatio;
    private double percentOutput;
    private double wheelCircumference;

    private MotorController m_motor;
    private RelativeEncoder m_encoder;

    public RotateMotorCommand(MotorController m_motor, RelativeEncoder m_encoder, double distance, double gearBoxRatio, double percentOutput, double wheelCircumference) {
        this.distance = distance;
        this.gearBoxRatio = gearBoxRatio;
        this.percentOutput = percentOutput;
        this.wheelCircumference = wheelCircumference;
        
        this.m_motor = m_motor;
        this.m_encoder = m_encoder;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_motor.set(percentOutput);
    }

    @Override
    public boolean isFinished() {
        if (distance <= m_encoder.getPosition() / gearBoxRatio * wheelCircumference) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_motor.set(0);
    }
}
