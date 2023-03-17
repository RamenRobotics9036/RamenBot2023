package frc.robot.Commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.ArmSystem;

public class SetArmPositionCommand extends CommandBase{
    private int position;
    private double rotations;
    private int direction = 1;//moving up or down
    private CANSparkMax winchMotor;
    private CANSparkMax armMotor;
    private RelativeEncoder winchEncoder;
    private RelativeEncoder armEncoder;


    
    public SetArmPositionCommand(int position, RelativeEncoder winchEncoder, CANSparkMax winchMotor){ //0 is default (all back all in), 1 is low rung, 2 is mid rung, 3 is "high"
        this.position = position;            
        this.winchEncoder = winchEncoder;
        this.winchMotor = winchMotor;
    }
    @Override
    public void initialize(){ // will set # of rotations needed for certain height

        if (position == 0){  
            rotations = 0; 
            direction = -1;
            
        } else if (position == 1){ // hybrid node
            rotations = 3;

        } else if (position == 2){ // middle node
            rotations = 6.4;

        } else{ // high node (if we can) 
            rotations = 8;
        }
        
        if (rotations - winchEncoder.getPosition() > 0){
            direction = 1;
        } else {
            direction = -1;
        }

    }
    @Override
    public void execute(){ //what to run
        winchMotor.set(direction*0.5);
    }
    @Override
    public boolean isFinished(){
        if (direction == 1 && winchEncoder.getPosition() >= rotations ){
            return true;
        } else if (direction == -1 && winchEncoder.getPosition() <= rotations){
            return true;
        }
        return false;
    }

    
}
