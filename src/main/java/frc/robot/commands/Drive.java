package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriverConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import java.util.Set;

public class Drive implements Command {
    public enum State {
        OpenLoop
    }
    
    private State state;
    Subsystem[] requirements = { RobotContainer.drivetrain };
    
    public Drive(State state){
        this.state = state;
    }
    
    @Override
    public void execute(){
        // Retrieving the deadbanded throttle and turn values (the controller joystick values)
        double throttle = RobotContainer.getThrottleValue();
        double turn = RobotContainer.getTurnValue();

        double left, right;
        
        switch (state) {
            // maybe more drive states later? idk
            case OpenLoop:
                // Differential drive as long as throttle is greater than zero (deadbanded).
                if (throttle != 0) {
                    left = (throttle + throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;
                    right = (throttle - throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;
                } else {
                    // Turns in place when there is no throttle input
                    left = turn * DriverConstants.kTurnInPlaceSens;
                    right = -turn * DriverConstants.kTurnInPlaceSens;
                }
    
                Drivetrain.setOpenLoop(left, right);
                break;
            default:
                break;
        }
    }
    
    // When this command ends, it stops the drivetrain to guarantee safety
    @Override
    public void end(boolean interrupted) {
        Drivetrain.stopMotors();
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
