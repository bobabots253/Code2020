package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;

public class Intake extends ProfiledPIDSubsystem {
    
    private static final TalonSRX armMotor = new TalonSRX(IntakeConstants.armMotor);
    private static final TalonSRX spinMotor = new TalonSRX(IntakeConstants.spinMotor);

    private static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kCos, IntakeConstants.kV, IntakeConstants.kA);
    
    private static Intake instance;
    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }
    
    private Intake(){
        super(new ProfiledPIDController(IntakeConstants.kP , IntakeConstants.kI, IntakeConstants.kD,
            new TrapezoidProfile.Constraints(IntakeConstants.kMaxVelocity, IntakeConstants.kMaxAcceleration)), 0);

        /* Settings for the intake arm motor */
        armMotor.configPeakCurrentLimit(45);
        armMotor.configPeakCurrentDuration(125);
        armMotor.configContinuousCurrentLimit(38);
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        armMotor.setSensorPhase(true);
        armMotor.setInverted(false);

        /* Settings for the intake spin motor */
        spinMotor.configPeakCurrentLimit(45);
        spinMotor.configPeakCurrentDuration(125);
        spinMotor.configContinuousCurrentLimit(38);
        spinMotor.setNeutralMode(NeutralMode.Coast);
        spinMotor.setInverted(false);

        /* Common intake motor settings */
        Arrays.asList(armMotor, spinMotor).forEach(motor -> {
            motor.configVoltageCompSaturation(12);
            motor.enableVoltageCompensation(true);
            motor.enableCurrentLimit(true);
        });
    }

    /**
     * Sets the intake to spin at a given voltage
     * 
     * @param value Percent of maximum voltage to send to motor
     */
    public void spin(double value) {
        spinMotor.set(ControlMode.PercentOutput, value);
    }

    /**
     * Resets encoder to zero
     */
    public void resetEncoder() {
        resetEncoder(0);
    }

    /**
     * Resets encoder to a specified value
     * @param value Value to set encoder position to
     */
    public void resetEncoder(int value) {
        armMotor.setSelectedSensorPosition(0);
    }

    /**
     * Returns the arm's current position as an encoder value
     */
    @Override
    public double getMeasurement() {
        return ticksToRadians(armMotor.getSelectedSensorPosition());
    }

    /**
     * Uses the value calculated by ProfiledPIDSubsystem
     */
    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate feedforward from the setpoint
        double feedforward = FEEDFORWARD.calculate(setpoint.position, setpoint.velocity);
        // Set motor, converting voltage to percent voltage
        armMotor.set(ControlMode.PercentOutput, (output + feedforward)/12.0);
    }

    /**
     * Converts encoder ticks to arm position in radians
     * 
     * @param value absolute encoder value
     * @return intake position in radians
     */
    private double ticksToRadians(double value) {
        return value * IntakeConstants.kRadPerTick;
    }
}
