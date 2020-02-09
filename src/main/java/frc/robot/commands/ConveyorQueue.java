package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.Conveyor;

public class ConveyorQueue implements Command {

    private Subsystem[] requirements = {Conveyor.getInstance()};

    public void execute() { 
        if(Conveyor.getInstance().getSensor()) {
            Conveyor.setOpenLoop(ConveyorConstants.kQueueSpeed);
        } else {
            Conveyor.setOpenLoop(0);
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