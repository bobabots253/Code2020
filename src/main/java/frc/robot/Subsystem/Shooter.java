/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystem;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Shooter implements Subsystem {
    private static final CANSparkMax shooterSparkMax = new CANSparkMax(Constants.ShooterConstants.shooterMotor, MotorType.kBrushless);
    private CANPIDController shootController = new CANPIDController(shooterSparkMax);
    private CANEncoder shooterEncoder = new CANEncoder(shooterSparkMax, EncoderType.kHallSensor, 42);
    private static Shooter instance;

    public static Shooter getInstance(){
        if(instance == null) instance = new Shooter();
        return instance;
    }
    
    public Shooter() {
        shooterSparkMax.enableVoltageCompensation(12);  // no touchy
        shooterSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
   
    }
    
    public void shoot(double speed){
        shooterSparkMax.set(speed);
    }
    
    public void stopMotors(){
        shooterSparkMax.stopMotor();
    }
}