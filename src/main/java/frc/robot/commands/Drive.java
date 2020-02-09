package frc.robot.commands;


import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import java.util.Set;

public class Drive implements Command {
    public enum State {
        CurvatureDrive2019, CheesyDriveOpenLoop, CheesyDriveClosedLoop
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
            
            case CurvatureDrive2019:
                // Differential drive as long as throttle is greater than zero (deadbanded).
                if (throttle != 0) {
                    left = (throttle + throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;
                    right = (throttle - throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;
                } else {
                    // Turns in place when there is no throttle input
                    left = turn * DriverConstants.kTurnInPlaceSens;
                    right = -turn * DriverConstants.kTurnInPlaceSens;
                }

                break;

            case CheesyDriveOpenLoop:
                if (throttle != 0) {
                    throttle *= DrivetrainConstants.kMaxSpeedMPS;
                    turn *= DriverConstants.kMaxCurvature * throttle;

                    DifferentialDriveWheelSpeeds wSpeeds = Drivetrain.KINEMATICS.toWheelSpeeds(new ChassisSpeeds(throttle, 0, turn));
                    wSpeeds.normalize(DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kDriveSens);

                    left = wSpeeds.leftMetersPerSecond / DrivetrainConstants.kMaxSpeedMPS;
                    right = wSpeeds.rightMetersPerSecond / DrivetrainConstants.kMaxSpeedMPS;
                } else {
                    // Turns in place when there is no throttle input
                    left = turn * DriverConstants.kTurnInPlaceSens;
                    right = -turn * DriverConstants.kTurnInPlaceSens;
                }

                break;

            case CheesyDriveClosedLoop:
                if (throttle != 0) {
                    throttle *= DrivetrainConstants.kMaxSpeedMPS;
                    turn *= DriverConstants.kMaxCurvature * throttle;

                    DifferentialDriveWheelSpeeds _wSpeeds = Drivetrain.KINEMATICS.toWheelSpeeds(new ChassisSpeeds(throttle, 0, turn));
                    _wSpeeds.normalize(DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kDriveSens);

                    left = Drivetrain.FEEDFORWARD.calculate(_wSpeeds.leftMetersPerSecond);
                    right = Drivetrain.FEEDFORWARD.calculate(_wSpeeds.rightMetersPerSecond);

                    left += Drivetrain.LEFT_PID_CONTROLLER.calculate(_wSpeeds.leftMetersPerSecond - Drivetrain.getLeftEncVelocityMeters());
                    right += Drivetrain.RIGHT_PID_CONTROLLER.calculate(_wSpeeds.rightMetersPerSecond - Drivetrain.getRightEncVelocityMeters());
                    
                    // Convert voltages to percent voltages
                    left /= 12;
                    right /= 12;
                } else {
                    // Turns in place when there is no throttle input
                    left = turn * DriverConstants.kTurnInPlaceSens;
                    right = -turn * DriverConstants.kTurnInPlaceSens;
                }
                
                break;

            default:
                left = 0;
                right = 0;
                break;
        }
        Drivetrain.setOpenLoop(left, right);
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
