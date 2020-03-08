package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.Conveyor;

import java.util.Set;

public class ConveyorQueue implements Command {

    private Subsystem[] requirements = {Conveyor.getInstance()};
    private PowerDistributionPanel pdp = new PowerDistributionPanel();

    public enum State {
        OneSensor, TwoSensors, None
    }

    private State state;

    Timer photoelectric_delay = new Timer();
    Timer current_spike = new Timer();

    public ConveyorQueue(State state) {
        this.state = state;
        photoelectric_delay.start();
        current_spike.start();
    }

    public void execute() { 
        if (Conveyor.getInstance().getHorizontalSensor()){
            if (Conveyor.getInstance().getVerticalSensor()){
                Conveyor.getInstance().setOpenLoop(0);
                
            } else {
                Conveyor.getInstance().setOpenLoop(0.45);
            } 
        } else {
            Conveyor.getInstance().setOpenLoop(0);
        }
    }

    public void end() {
        Conveyor.getInstance().stop();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }    
}
