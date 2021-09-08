package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Util;

public class Climber implements Subsystem {
    Servo leftServo, rightServo;

    private static Climber instance;
    public static TalonSRX
         leftMotor = Util.createTalonSRX(ClimberConstants.leftMotorID, true),
         rightMotor = Util.createTalonSRX(ClimberConstants.rightMotorID, true); 

    private Climber(){
        leftMotor.setInverted(false);
        rightMotor.setInverted(false);
        leftServo = new Servo(ClimberConstants.leftServoID);
        rightServo = new Servo(ClimberConstants.rightServoID);
        leftMotor.clearStickyFaults();
        rightMotor.clearStickyFaults();
    }
    public void leftServoUp() {
        leftServo.set(0.25);
    }

    public void rightServoUp() {
        rightServo.set(0.66);
    }

    public void leftServoDown() {
        leftServo.set(0.75);
    }

    public void rightServoDown() {
        rightServo.set(0.17);
    }

    public void servosDown() {
        leftServoDown();
        rightServoDown();
    }
    
    public void servosUp() {
        leftServoUp();
        rightServoUp();
    }
    public void climbRight(double rightSpeed) {
        rightMotor.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void climbLeft(double leftSpeed) {
        leftMotor.set(ControlMode.PercentOutput, -leftSpeed);
    }
    public void climbUnity(double speed) {
        climbLeft(speed);
        climbRight(speed);
    }
    public static Climber getInstance(){
        if(instance == null) {instance = new Climber();}
        return instance;
    }
    
    public void stopMotors(){
        stopLeftMotor();
        stopRightMotor();
    }

    public void stopLeftMotor(){
        leftServoUp();
        leftMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
    public void stopRightMotor(){
        rightServoUp();
        rightMotor.set(ControlMode.PercentOutput, 0.0);
    }
}
