package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ShooterConstants;

public class Shooter implements Subsystem {
    private static final CANSparkMax motor = new CANSparkMax(ShooterConstants.motorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static CANPIDController pidController;

    private static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
    
    private static Shooter instance;
    public static Shooter getInstance(){
        if (instance == null) instance = new Shooter();
        return instance;
    }
    
    private Shooter(){
        motor.enableVoltageCompensation(12);
    
        pidController = motor.getPIDController();
        pidController.setP(ShooterConstants.kP, ShooterConstants.kSlotID);
        pidController.setI(ShooterConstants.kI, ShooterConstants.kSlotID);
        pidController.setD(ShooterConstants.kD, ShooterConstants.kSlotID);
        pidController.setOutputRange(ShooterConstants.kMin, ShooterConstants.kMax, ShooterConstants.kSlotID);
    }

    /**
     * Sets the speed of the shooter in percent of max voltage (overriding the closed loop velocity control)
     * @param value percent of voltage [-1, 1]
     */
    public static void setOpenLoop(double value){
        motor.set(value);
    }
    
    /**
     * Stops the motor
     */
    public static void stop(){
        motor.stopMotor();
    }
    
    /**
     * Set the setpoint of the flywheel based on an RPM target
     * @param RPM the RPM for the flywheel to spin at
     */
    public void setGoal(double RPM){
        pidController.setReference(RPM, ControlType.kVelocity, ShooterConstants.kSlotID, FEEDFORWARD.calculate(RPM), CANPIDController.ArbFFUnits.kVoltage);
    }
}
