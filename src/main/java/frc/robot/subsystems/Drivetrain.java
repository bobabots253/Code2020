package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Arrays;

public class Drivetrain implements Subsystem {
    private static final TalonSRX
        leftMaster = new TalonSRX(Constants.DriveTrainConstants.leftMaster),
        leftSlave = new TalonSRX(Constants.DriveTrainConstants.leftSlave),
        rightMaster = new TalonSRX(Constants.DriveTrainConstants.rightMaster),
        rightSlave = new TalonSRX(Constants.DriveTrainConstants.rightSlave);

    private static Drivetrain instance;

    public static Drivetrain getInstance(){
        if (instance == null) instance = new Drivetrain();
        return instance;
    }
    
    private Drivetrain(){
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        
        // inversion on opposite sides of the drivetrain
        Arrays.asList(leftMaster, leftSlave).forEach(motor -> motor.setInverted(true));
        Arrays.asList(rightMaster, rightSlave).forEach(motor -> motor.setInverted(false));
        
        // Motor settings
        Arrays.asList(leftMaster, leftSlave, rightMaster, rightSlave).forEach(motor -> {
            motor.configPeakCurrentLimit(45);
            motor.configPeakCurrentDuration(125);
            motor.configContinuousCurrentLimit(38);
            motor.enableCurrentLimit(true);
            
            motor.configVoltageCompSaturation(12, 10);
            motor.enableVoltageCompensation(true);
            motor.configClosedloopRamp(0.05, 0);
            motor.setNeutralMode(NeutralMode.Brake);
        });
    
        /* Encoder settings */
        Arrays.asList(leftMaster, rightMaster).forEach(motor -> {
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1, 10);
            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
            motor.setSensorPhase(false);
        });
        
        register();
    }

    /**
     * Sets drivetrain speeds in open loop (% of max voltage)
     *
     * @param left   Percent output of motors on left side of drivetrain
     * @param right  Percent output of motors on right side of drivetrain
     */
    public static void setOpenLoop(Double left, Double right){
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    
        SmartDashboard.putNumber("left volts", left);
        SmartDashboard.putNumber("right volts", right);
    }
    
    /**
     * Stops the drivetrain motors by setting their speed to 0
     */
    public static void stopMotors(){
        setOpenLoop(0.0, 0.0);
    }
    
    /* Static class to contain the speeds of each side of the drivetrain */
    public static class WheelState {
        public double left, right;
        
        public WheelState(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }
}
