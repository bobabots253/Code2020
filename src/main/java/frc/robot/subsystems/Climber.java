package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Util;

public class Climber implements Subsystem {
    private static Climber instance;
    private static TalonSRX leftMotor = new TalonSRX(Constants.ClimberConstants.leftMotorID);
    private static TalonSRX rightMotor = new TalonSRX(Constants.ClimberConstants.rightMotorID);

    /* public static TalonSRX
         leftMotor = Util.createTalonSRX(Constants.ClimberConstants.leftMotorID, true),
         rightMotor = Util.createTalonSRX(Constants.ClimberConstants.rightMotorID, true); 
*/
    private Climber(){
        leftMotor.setInverted(false);
        rightMotor.setInverted(false);
    }

    public void climbUnity(double speed){
        System.out.println("here: " + speed);
        leftMotor.set(ControlMode.PercentOutput, speed);
        rightMotor.set(ControlMode.PercentOutput, speed);

        double maxLeft = 0;
        double maxRight = 0;
        if(leftMotor.getSupplyCurrent() > maxLeft){
            maxLeft = leftMotor.getSupplyCurrent();
        }
        if(rightMotor.getSupplyCurrent() > maxRight){
            maxRight = rightMotor.getSupplyCurrent();
        }
        SmartDashboard.putNumber("Left Climber Current", maxLeft);
        SmartDashboard.putNumber("Right Climber Current", maxRight);
        
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