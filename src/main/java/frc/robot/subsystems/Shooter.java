package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Util;
import frc.robot.Constants.ShooterConstants;

public class Shooter implements Subsystem {
    /*private static final CANSparkMax master = Util.createSparkMAX(ShooterConstants.master, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax slave = Util.createSparkMAX(ShooterConstants.slave, CANSparkMaxLowLevel.MotorType.kBrushless);*/

    private static final CANSparkMax master = new CANSparkMax(ShooterConstants.master, MotorType.kBrushless);
    private static final CANSparkMax slave = new CANSparkMax(ShooterConstants.slave, MotorType.kBrushless);

    private static CANPIDController pidController;

    private static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
    
    private static Shooter instance;
    public static Shooter getInstance(){
        if (instance == null) instance = new Shooter();
        return instance;
    }
    
    private Shooter(){

        master.restoreFactoryDefaults();
        slave.restoreFactoryDefaults();

        master.enableVoltageCompensation(12);
        slave.enableVoltageCompensation(12);

        master.setIdleMode(IdleMode.kCoast);
        slave.setIdleMode(IdleMode.kCoast);

        master.setInverted(false);
        slave.follow(master, true);

        pidController = master.getPIDController();
        pidController.setP(ShooterConstants.kP, ShooterConstants.kSlotID);
        pidController.setI(ShooterConstants.kI, ShooterConstants.kSlotID);
        pidController.setD(ShooterConstants.kD, ShooterConstants.kSlotID);
        pidController.setOutputRange(ShooterConstants.kMin, ShooterConstants.kMax, ShooterConstants.kSlotID);

        master.burnFlash();
        slave.burnFlash();
    }

    /**
     * Sets the speed of the shooter in percent of max voltage (overriding the closed loop velocity control)
     * @param value percent of voltage [-1, 1]
     */
    public void setOpenLoop(double value){
        master.set(value);

    if(value != 0){
        //goShooter = true;

    }
       
    }
    
    /**
     * Stops the motor
     */
    public void stop(){
        master.stopMotor();
        
    }
    
    /**
     * Set the setpoint of the flywheel based on an RPM target
     * @param RPM the RPM for the flywheel to spin at
     */
    public void setGoal(double RPM){
        pidController.setReference(RPM, ControlType.kVelocity, ShooterConstants.kSlotID, FEEDFORWARD.calculate(RPM), CANPIDController.ArbFFUnits.kVoltage);
    }
}
