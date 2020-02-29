package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Units;
import frc.robot.Util;
import frc.robot.Constants.IntakeConstants;

public class Intake extends ProfiledPIDSubsystem {
    
    private static final TalonSRX armMotor = Util.createTalonSRX(IntakeConstants.armMotor, false);
    private static final TalonSRX spinMotor = Util.createTalonSRX(IntakeConstants.spinMotor, false);
    private static final CANSparkMax conveyorMotor = Util.createSparkMAX(IntakeConstants.conveyorMotor, MotorType.kBrushless);

    private static final DutyCycleEncoder armEncoder = new DutyCycleEncoder(1);

    private static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kCos, IntakeConstants.kV, IntakeConstants.kA);
    
    
    private static Intake instance;
    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }

    /**
     * Enum class representing the two possible positions of the intake arm, UP and DOWN
     */
    public enum State {
        UP(0), DOWN(0);

        public double position;

        /**
         * @param position the value of the arm position in radians
         */
        private State(double position) {
            this.position = position;
        }
    }

    private Intake(){
        super(new ProfiledPIDController(IntakeConstants.kP , IntakeConstants.kI, IntakeConstants.kD,
            new TrapezoidProfile.Constraints(IntakeConstants.kMaxVelocity, IntakeConstants.kMaxAcceleration)), 0);    

        armEncoder.setDistancePerRotation(8128);

        conveyorMotor.setInverted(true);
        conveyorMotor.burnFlash();

        setGoal(IntakeConstants.kOffsetRadians);

        register();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder dist", armEncoder.getDistance());
    }

    /**
     * Sets the conveyor to spin at a percent of max speed
     * @param speed Percent speed
     */
    public void setConveyor(double value) {
        conveyorMotor.set(value);
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
    public void stop() {
        armMotor.set(ControlMode.PercentOutput, 0);
        spinMotor.set(ControlMode.PercentOutput, 0);
        conveyorMotor.set(0);
    }

    /**
     * Resets encoders to zero
     */
    public void resetEncoders() {
        resetEncoders(0);
    }

    /**
     * Resets encoder to a specified value
     * 
     * @param value Value to set encoder position to
     */
    public void resetEncoders(int value) {
        armEncoder.reset();
    }

    /**
     * Sets the motion profile's goal to a position represented by a State
     * 
     * @param state Goal to set
     */
    public void setGoal(State state) {
        setGoal(state.position);
    }

    /**
     * @return the arm's current position as a radian measure
     */
    @Override
    public double getMeasurement() {
        return armEncoder.getDistance() * 2 * Math.PI;
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
