package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Util;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor implements Subsystem {
    private static final CANSparkMax master = Util.createSparkMAX(ConveyorConstants.motor, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final DigitalInput photoelectric = new DigitalInput(0);

    private static Conveyor instance;
    public static Conveyor getInstance() {
        if (instance == null) instance = new Conveyor();
        return instance;
    }
    
    private Conveyor() {

        master.setInverted(true);
        master.burnFlash();

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
    public void setOpenLoop(double value) {
        master.set(value);
    }

    public void stop(){
        setOpenLoop(0);
    }
}
