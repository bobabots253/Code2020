package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

public class ArmDown implements Command {

    double armSpeed;
    private Subsystem[] requirements = {Intake.getInstance()};


    
    @Override
    public void execute() {
        if(Intake.armMotor.getStatorCurrent() > 30){
            armSpeed =0.05;
        } else {
            armSpeed = 0.5;
        }
        
        Intake.rotate(armSpeed);
    
    }

    @Override
    public void end(boolean interrupted) {
        Intake.stopMotors();
        //Command.super.end(interrupted);
    }
    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
        
    }

}