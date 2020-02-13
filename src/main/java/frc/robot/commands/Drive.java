package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.WheelState;

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

        WheelState wheelState;
        
        switch (state) {
            
            case CurvatureDrive2019:
                // Differential drive as long as throttle is greater than zero (deadbanded).
                wheelState = CurvatureDrive2019(throttle, turn);
                break;

            case CheesyDriveOpenLoop:
                wheelState = CheesyDriveOpenLoop(throttle, turn);
                break;

            case CheesyDriveClosedLoop:
                wheelState = CheesyDriveClosedLoop(throttle, turn);
                break;

            default:
                wheelState = new WheelState(0, 0);
                break;
        }
        Drivetrain.setOpenLoop(wheelState.left, wheelState.right);
    }
    
    public static WheelState CurvatureDrive2019(double throttle, double turn) {
        double left, right;
        
        if (throttle != 0) {
            left = (throttle + throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;
            right = (throttle - throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;
        } else {
            return TurnInPlace(turn);
        }
        
        return new WheelState(left, right);
    }
    
    public static WheelState CheesyDriveOpenLoop(double throttle, double turn) {
        double left, right;
        
        if (throttle != 0) {
            throttle *= DrivetrainConstants.kMaxSpeedMPS;
            turn *= DriverConstants.kMaxCurvature * throttle;
        
            DifferentialDriveWheelSpeeds wSpeeds = Drivetrain.KINEMATICS.toWheelSpeeds(new ChassisSpeeds(throttle, 0, turn));
            wSpeeds.normalize(DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kDriveSens);
        
            left = wSpeeds.leftMetersPerSecond / DrivetrainConstants.kMaxSpeedMPS + Drivetrain.FEEDFORWARD.calculate(wSpeeds.leftMetersPerSecond) / Constants.kMaxVoltage;
            right = wSpeeds.rightMetersPerSecond / DrivetrainConstants.kMaxSpeedMPS + Drivetrain.FEEDFORWARD.calculate(wSpeeds.rightMetersPerSecond) / Constants.kMaxVoltage;
        
        } else {
            return TurnInPlace(turn);
        }
        
        return new WheelState(left, right);
    }
    
    public static WheelState CheesyDriveClosedLoop(double throttle, double turn) {
        double left, right;
        
        if (throttle != 0) {
            throttle *= DrivetrainConstants.kMaxSpeedMPS;
            turn *= DriverConstants.kMaxCurvature * throttle;
        
            DifferentialDriveWheelSpeeds wSpeeds = Drivetrain.KINEMATICS.toWheelSpeeds(new ChassisSpeeds(throttle, 0, turn));
            wSpeeds.normalize(DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kDriveSens);
        
            left = Drivetrain.FEEDFORWARD.calculate(wSpeeds.leftMetersPerSecond);
            right = Drivetrain.FEEDFORWARD.calculate(wSpeeds.rightMetersPerSecond);
        
            left += Drivetrain.LEFT_PID_CONTROLLER.calculate(Drivetrain.getLeftEncVelocityMeters(), wSpeeds.leftMetersPerSecond);
            right += Drivetrain.RIGHT_PID_CONTROLLER.calculate(Drivetrain.getRightEncVelocityMeters(), wSpeeds.rightMetersPerSecond);
        
            // Convert voltages to percent voltages
            left /= Constants.kMaxVoltage;
            right /= Constants.kMaxVoltage;
        } else {
            return TurnInPlace(turn);
        }
        
        return new WheelState(left, right);
    }
    
    public static WheelState TurnInPlace(double turn) {
        double x = turn * DriverConstants.kTurnInPlaceSens;
        return new WheelState(x, -x);
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
