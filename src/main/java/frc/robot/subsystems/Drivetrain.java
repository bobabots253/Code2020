package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Arrays;

public class Drivetrain implements Subsystem {
    private static final TalonFX
        leftMaster = new TalonFX(Constants.DriveTrainConstants.leftMaster),
        leftSlave = new TalonFX(Constants.DriveTrainConstants.leftSlave),
        rightMaster = new TalonFX(Constants.DriveTrainConstants.rightMaster),
        rightSlave = new TalonFX(Constants.DriveTrainConstants.rightSlave);

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
        TalonFXConfiguration falconConfig = new TalonFXConfiguration();
        falconConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 45, 38, 0.125);
        Arrays.asList(leftMaster, leftSlave, rightMaster, rightSlave).forEach(motor -> {
            
            /*  TalonSRX configs
            motor.configPeakCurrentLimit(45);
            motor.configPeakCurrentDuration(125);
            motor.configContinuousCurrentLimit(38);
            motor.enableCurrentLimit(true);
            */
            
            motor.configAllSettings(falconConfig);
            
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
    
    /**
     * Zeroes encoders
     */
    public void resetEncoders() {
        resetEncoders(0, 0);
    }
    
    /**
     * Sets encoders to a specific value
     * @param left  left wheel value
     * @param right right wheel value
     */
    public void resetEncoders(int left, int right) {
        rightMaster.setSelectedSensorPosition(right);
        leftMaster.setSelectedSensorPosition(left);
    }
    
    /**
     * @return the current measurement of the left drivetrain encoder
     */
    public static double getLeftEnc() {
        return leftMaster.getSelectedSensorPosition();
    }
    
    /**
     * @return the current measurement of the right drivetrain encoder
     */
    public static double getRightEnc() {
        return rightMaster.getSelectedSensorPosition();
    }
    
    /**
     * @return the current velocity measurement of the left drivetrain encoder
     */
    public static double getLeftEncVelocity() {
        return leftMaster.getSelectedSensorVelocity();
    }
    
    /**
     * @return the current velocity measurement of the right drivetrain encoder
     */
    public static double getRightEncVelocity() {
        return rightMaster.getSelectedSensorVelocity();
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
