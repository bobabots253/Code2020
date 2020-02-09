package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends PIDSubsystem {
    private static final CANSparkMax motor = new CANSparkMax(ShooterConstants.motorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static CANEncoder encoder;

    private static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
    
    private static Shooter instance;
    public static Shooter getInstance(){
        if (instance == null) instance = new Shooter();
        return instance;
    }
    
    private Shooter(){
        super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
        getController().setTolerance(ShooterConstants.kTolerance);

        motor.enableVoltageCompensation(12);
        encoder = motor.getEncoder();
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
     * Uses the value calculated by PIDSubsystem
     */
    @Override
    protected void useOutput(double output, double setpoint) {
        motor.setVoltage(output + FEEDFORWARD.calculate(setpoint));

    }

    /**
     * Returns the encoder velocity (RPM)
     */
    @Override
    protected double getMeasurement() {
        return encoder.getVelocity();
    }
}
