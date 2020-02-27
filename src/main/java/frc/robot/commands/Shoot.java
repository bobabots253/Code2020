package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter;

import java.util.Set;

public class Shoot implements Command {
    
    private Subsystem[] requirements = {};
    
    @Override
    public void execute(){
        //TODO: implement
    }
    
    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stop();
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
