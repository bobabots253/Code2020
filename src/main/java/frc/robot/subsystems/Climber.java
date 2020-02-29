package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Climber implements Subsystem{
    private static Climber instance;
    public static TalonSRX
         leftMotor = new TalonSRX(Constants.ClimberConstants.leftMotorID),
         rightMotor = new TalonSRX(Constants.ClimberConstants.rightMotorID); 
    
    DutyCycleEncoder armEncoder;
    
    boolean connected;
    int frequency;
    double output, distance;


    private Climber(){

        armEncoder = new DutyCycleEncoder(0); //TODO: Set correct DIO Port
        armEncoder.setDistancePerRotation(0); //TODO: Scale the encoder to a angular value, accounting for gear ratio

        connected = armEncoder.isConnected();
        frequency = armEncoder.getFrequency();
        output = armEncoder.get();
        distance = armEncoder.getDistance();

    }

    @Override

    public void periodic(){
        SmartDashboard.putBoolean("Connected?", connected);
        SmartDashboard.putNumber("Frequency", frequency);
        SmartDashboard.putNumber("Output", output);
        SmartDashboard.putNumber("Distance", distance);

    }
    public static void climbUnity(double speed){
        leftMotor.set(ControlMode.PercentOutput, speed);
        rightMotor.set(ControlMode.PercentOutput, speed);
    }
    
    public static void climbRight(double rightSpeed){
        rightMotor.set(ControlMode.PercentOutput, rightSpeed);
    }

    public static void climbLeft(double leftSpeed){
        leftMotor.set(ControlMode.PercentOutput, leftSpeed);
    }

    public static Climber getInstance(){
        if(instance == null) {instance = new Climber();}
        return instance;
    }
    public static void stopMotors(){
        leftMotor.set(ControlMode.PercentOutput, 0.0);
        rightMotor.set(ControlMode.PercentOutput, 0.0);
    }
    public static void stopLeftMotor(){
        leftMotor.set(ControlMode.PercentOutput, 0.0);
    }
    public static void stopRightMotor(){
        rightMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
}