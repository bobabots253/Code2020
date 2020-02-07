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
    
    private Conveyor(){
    
    }
}
