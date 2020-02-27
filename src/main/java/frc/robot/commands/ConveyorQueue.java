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
        OneSensor, TwoSensors
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

        switch(state){
            case OneSensor:
                if(Conveyor.getInstance().getQueueSensor()) photoelectric_delay.reset();
                if(pdp.getCurrent(5) > 25 || pdp.getCurrent(10) > 25) current_spike.reset();
                
                if(current_spike.hasElapsed(0.25)) {
                    if(!photoelectric_delay.hasElapsed(0.1)){
                        Conveyor.getInstance().setOpenLoop(ConveyorConstants.kQueueSpeed);
                    } else {
                        Conveyor.getInstance().stop();
                    }
                } else {
                    Conveyor.getInstance().setOpenLoop(-ConveyorConstants.kQueueSpeed);
                }

                break;
            case TwoSensors:
                if(!Conveyor.getInstance().getShooterSensor()) {
                    if(Conveyor.getInstance().getQueueSensor()) {
                        Conveyor.getInstance().setOpenLoop(ConveyorConstants.kQueueSpeed);
                    } else {
                        Conveyor.getInstance().stop();
                    }
                } else {
                    Conveyor.getInstance().stop();
                }
                break;
            default:
                break;
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
