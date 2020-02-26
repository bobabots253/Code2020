package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Units;

import java.util.List;

public class Intake extends ProfiledPIDSubsystem {
    
    private static final TalonSRX armMotor = new TalonSRX(IntakeConstants.armMotor);
    private static final TalonSRX spinMotor = new TalonSRX(IntakeConstants.spinMotor);

    private static final CANSparkMax conveyorMotor = new CANSparkMax(ConveyorConstants.slave_MotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

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
        List.of(armMotor, spinMotor).forEach(motor -> {
            motor.configVoltageCompSaturation(Constants.kMaxVoltage);
            motor.enableVoltageCompensation(true);
            motor.enableCurrentLimit(true);
        });

        // conveyor motor

        conveyorMotor.clearFaults();

        conveyorMotor.restoreFactoryDefaults();

        conveyorMotor.enableVoltageCompensation(Constants.kMaxVoltage);

        conveyorMotor.setIdleMode(IdleMode.kBrake);

        conveyorMotor.setInverted(true);

        // burn flash LAST
        conveyorMotor.burnFlash();
    }

    /**
     * Sets the conveyor to spin at a percent of max speed
     * @param speed Percent speed
     */
    public void setConveyor(double speed) {
        conveyorMotor.set(speed);
    }

    /**
     * Sets the intake to spin at a given voltage
     * 
     * @param value Percent of maximum voltage to send to motor
     */
    public void intake(double value) {
        spinMotor.set(ControlMode.PercentOutput, value);
    }

    /**
     * Set the intake to rotate manually (overriding the position control)
     * @param value Percent of maximum voltage to send to motor
     */
    public void rotate(double value) {
        armMotor.set(ControlMode.PercentOutput, value);
    }

    /**
     * Stops the motors 
     */
    public void stopMotors() {
        armMotor.set(ControlMode.PercentOutput, 0);
        spinMotor.set(ControlMode.PercentOutput, 0);
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
     * @return the arm's current position as an encoder value
     */
    @Override
    public double getMeasurement() {
        return Units.IntakeUnits.TicksToRadians(armMotor.getSelectedSensorPosition());
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
}
