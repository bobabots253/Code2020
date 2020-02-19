package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor implements Subsystem {
    private static final CANSparkMax motor = new CANSparkMax(ConveyorConstants.motorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    private static Conveyor instance;
    
    public static Conveyor getInstance() {
        if (instance == null) instance = new Conveyor();
        return instance;
    }
    
    private Conveyor() {
        motor.enableVoltageCompensation(Constants.kMaxVoltage);
        motor.setInverted(false);
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
        motor.set(value);
    }

    public static void stop(){
        setOpenLoop(0);
    }
}
