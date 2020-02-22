package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor implements Subsystem {
    private static final CANSparkMax master = new CANSparkMax(ConveyorConstants.master_MotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax slave = new CANSparkMax(ConveyorConstants.slave_MotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    private static Conveyor instance;
    
    public static Conveyor getInstance() {
        if (instance == null) instance = new Conveyor();
        return instance;
    }
    
    private Conveyor() {
        master.restoreFactoryDefaults();
        slave.restoreFactoryDefaults();

        slave.follow(master, true);

        master.enableVoltageCompensation(Constants.kMaxVoltage);
        slave.enableVoltageCompensation(Constants.kMaxVoltage);

        master.burnFlash();
        slave.burnFlash();
    }
    /**
     * Determine whether a power cell is seen by the queuing sensor at the beginning of the conveyor
     * 
     * @return true if the sensor sees a ball, else: false
     */
    public boolean getQueueSensor() {
        // TODO: implement
        return true;
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
