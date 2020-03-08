package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LEDConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LED;

public class LEDCommand implements Command {
    private Subsystem[] requirements = {RobotContainer.led};
    LEDConstants.State state;
    public Color color;
  
    public LEDCommand(LEDConstants.State state){
        this.state = state;
    }

    
    @Override
    public void execute(){
        switch(state){
            case CLIMBING:
            LED.getInstance().setColor(Color.kBlack);
            
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        LED.getInstance().setColor(Color.kWhite);
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
    
}