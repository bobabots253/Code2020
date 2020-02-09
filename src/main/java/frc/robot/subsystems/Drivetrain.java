package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Arrays;

public class Drivetrain implements Subsystem {
    private static final TalonFX
        leftMaster = new TalonFX(Constants.DrivetrainConstants.leftMaster),
        leftSlave = new TalonFX(Constants.DrivetrainConstants.leftSlave),
        rightMaster = new TalonFX(Constants.DrivetrainConstants.rightMaster),
        rightSlave = new TalonFX(Constants.DrivetrainConstants.rightSlave);
    
    public static final TalonFX[] motors = {leftMaster, leftSlave, rightMaster, rightSlave};

    public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(DrivetrainConstants.trackWidth);
    public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA);
    public static final PIDController LEFT_PID_CONTROLLER = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
    public static final PIDController RIGHT_PID_CONTROLLER = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);

    private static Drivetrain instance;

    public static Drivetrain getInstance(){
        if (instance == null) instance = new Drivetrain();
        return instance;
    }
    
    private Drivetrain(){
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        
        // inversion on opposite sides of the drivetrain
        Arrays.asList(leftMaster, leftSlave).forEach(motor -> motor.setInverted(false));
        Arrays.asList(rightMaster, rightSlave).forEach(motor -> motor.setInverted(true));
        
        // Motor settings
        TalonFXConfiguration falconConfig = new TalonFXConfiguration();
        falconConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            DrivetrainConstants.kLimitEnabled,
            DrivetrainConstants.kCurrentLimit,
            DrivetrainConstants.kTriggerThresholdCurrent,
            DrivetrainConstants.kTriggerThresholdTimeDelta
        );
        Arrays.asList(motors).forEach(motor -> {
            
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

    public static double getLeftEncVelocityMeters() {
        return ticksPerDecisecondToMetersPerSecond(getLeftEncVelocity());
    }

    public static double getRightEncVelocityMeters() {
        return ticksPerDecisecondToMetersPerSecond(getRightEncVelocity());
    }

    public static double ticksPerDecisecondToMetersPerSecond(double ticksPerDecisecond){
        return (ticksPerDecisecond * 10 * Math.PI * DrivetrainConstants.wheelDiameter) / DrivetrainConstants.ticksPerRotation;
    }

    public static double metersPerSecondToTicksPerDecisecond(double metersPerSecond){
        return metersPerSecond * DrivetrainConstants.ticksPerRotation / (10 * Math.PI * DrivetrainConstants.wheelDiameter);
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
