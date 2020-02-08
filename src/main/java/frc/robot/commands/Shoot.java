package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

import java.util.Set;

public class Shoot implements Command {
    
    private Subsystem[] requirements = { RobotContainer.conveyor, RobotContainer.shooter };
    
    @Override
    public void execute(){
    }
    
    @Override
    public void end(boolean interrupted) {
        Shooter.stop();
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}