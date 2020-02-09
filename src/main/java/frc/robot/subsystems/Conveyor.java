package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor implements Subsystem {
    private static final CANSparkMax motor = new CANSparkMax(ConveyorConstants.motorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    private static Conveyor instance;
    
    public static Conveyor getInstance() {
        if (instance == null) instance = new Conveyor();
        return instance;
    }
    
    private Conveyor() {
        motor.enableVoltageCompensation(12);
        motor.setInverted(false);
    }

    public boolean getSensor() {
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

}
