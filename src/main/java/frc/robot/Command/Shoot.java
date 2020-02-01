
package frc.robot.Command;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.*;

public class Shoot implements Command {
    
    Subsystem[] requirements = { RobotContainer.shooter };
    Shooter shooter = RobotContainer.shooter;
    
    public Shoot() {
        
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = RobotContainer.getJoystickYValue();
        shooter.shoot(speed);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stopMotors();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}