package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Conveyor;

import java.util.Set;

public class ConveyorQueue implements Command {

    private static final Set<Subsystem> requirements = Set.of(Conveyor.getInstance());

    public ConveyorQueue() {
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
        return requirements;
    }    
}
