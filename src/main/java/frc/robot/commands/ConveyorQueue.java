package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.Conveyor;

import java.util.Set;

public class ConveyorQueue implements Command {

    private Subsystem[] requirements = {Conveyor.getInstance()};

    public enum State {
        OneSensor, TwoSensors
    }

    private State state;

    public ConveyorQueue(State state) {
        this.state = state;
    }

    public void execute() { 

        switch(state){
            case OneSensor:
                if(Conveyor.getInstance().getQueueSensor()) {
                    Conveyor.setOpenLoop(ConveyorConstants.kQueueSpeed);
                } else {
                    Conveyor.stop();
                }
                break;
            case TwoSensors:
                if(!Conveyor.getInstance().getShooterSensor()) {
                    if(Conveyor.getInstance().getQueueSensor()) {
                        Conveyor.setOpenLoop(ConveyorConstants.kQueueSpeed);
                    } else {
                        Conveyor.stop();
                    }
                } else {
                    Conveyor.stop();
                }
                break;
            default:
                break;
        }
    }

    public void end() {
        Conveyor.setOpenLoop(0);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
    
}