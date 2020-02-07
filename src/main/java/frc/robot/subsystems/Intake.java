package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;

public class Intake implements Subsystem {
    public static final ProfiledPIDController pidController = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD,
            new TrapezoidProfile.Constraints());  //TODO: Aaron do this
    
    private static final TalonSRX armMotor = new TalonSRX(IntakeConstants.armMotor);
    private static final TalonSRX spinMotor = new TalonSRX(IntakeConstants.spinMotor);
    
    private static Intake instance;
    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }
    
    private Intake(){
    
    }
}
