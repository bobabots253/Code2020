package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;

import java.util.List;

public class Conveyor implements Subsystem {
    public static final CANSparkMax master = new CANSparkMax(ConveyorConstants.master_MotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final CANSparkMax slave = new CANSparkMax(ConveyorConstants.slave_MotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final DigitalInput photoelectric = new DigitalInput(0);

    private static Conveyor instance;
    public static Conveyor getInstance() {
        if (instance == null) instance = new Conveyor();
        return instance;
    }
    
    private Conveyor() {
        List.of(master, slave).forEach(motor -> {
            motor.clearFaults();
            motor.restoreFactoryDefaults();
            motor.enableVoltageCompensation(Constants.kMaxVoltage);
            motor.setIdleMode(IdleMode.kBrake);
            motor.burnFlash();
        });

        master.setInverted(true);
        slave.follow(master, true);

        register();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Queue Sensor", getQueueSensor());
    }
    /**
     * Determine whether a power cell is seen by the queuing sensor at the beginning of the conveyor
     * 
     * @return true if the sensor sees a ball, else: false
     */
    public boolean getQueueSensor() {
        return (photoelectric.get());
    }   

    /**
     * Determine whether a power cell is seen by the shooter sensor at the end of the conveyor
     * 
     * @return true if the sensor sees a ball, else: false
     */
    public boolean getShooterSensor() {
        // TODO: implement
        return true;
    }

    /**
     * Sets the conveyor in percent of max speed
     * @param value Percent speed
     */
    public static void setOpenLoop(double value) {
        master.set(value);
       
    }

    public static void stop(){
        setOpenLoop(0);
    }
}
