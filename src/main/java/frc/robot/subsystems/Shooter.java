package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ShooterConstants;

public class Shooter implements Subsystem {
    private static final CANSparkMax motor = new CANSparkMax(ShooterConstants.motorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    private static Shooter instance;
    public static Shooter getInstance(){
        if (instance == null) instance = new Shooter();
        return instance;
    }
    
    private Shooter(){
    
    }
    
    public static void setOpenLoop(double speed){
        motor.set(speed);
    }
    
    public static void stop(){
        motor.stopMotor();
    }
}
