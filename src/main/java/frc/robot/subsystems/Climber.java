package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Util;

public class Climber implements Subsystem {
    private static Climber instance;
    public static TalonSRX
         leftMotor = Util.createTalonSRX(Constants.ClimberConstants.leftMotorID, true),
         rightMotor = Util.createTalonSRX(Constants.ClimberConstants.rightMotorID, true);
    
    private static Servo leftServo, rightServo;

    private Climber(){
        leftMotor.setInverted(false);
        rightMotor.setInverted(false);
        leftServo = new Servo(Constants.ClimberConstants.leftServoID);
        rightServo = new Servo(Constants.ClimberConstants.rightServoID);
    }
    
    /**
     * Set the servos' position.
     * Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
     * @param value Position from 0.0 to 1.0.
     */
    public void setServos(double value) {
        leftServo.set(value);
        rightServo.set(value);
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

    public static Climber getInstance(){
        if(instance == null) {instance = new Climber();}
        return instance;
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