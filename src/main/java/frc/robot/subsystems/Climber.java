package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Climber implements Subsystem{
    private static Climber instance;
    private static TalonSRX
         leftMotor = new TalonSRX(Constants.ClimberConstants.leftMotorID),
         rightMotor = new TalonSRX(Constants.ClimberConstants.rightMotorID); 
    

    private Climber(){

    }
    public static Climber getInstance(){
        if(instance == null) {instance = new Climber();}
        return instance;
    }
    public void climbUnity(double speed){
        leftMotor.set(ControlMode.PercentOutput, speed);
        rightMotor.set(ControlMode.PercentOutput, speed);
    }
    
    public void climbRight(double rightSpeed){
        rightMotor.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void climbLeft(double leftSpeed){
        leftMotor.set(ControlMode.PercentOutput, leftSpeed);
    }

    public void stopMotors(){
        leftMotor.set(ControlMode.PercentOutput, 0.0);
        rightMotor.set(ControlMode.PercentOutput, 0.0);
    }
    public void stopLeftMotor(){
        leftMotor.set(ControlMode.PercentOutput, 0.0);
    }
    public void stopRightMotor(){
        rightMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
}