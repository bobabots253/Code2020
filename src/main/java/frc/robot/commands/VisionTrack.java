package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.LEDMode;
import frc.robot.RobotContainer.VisionPipeline;
import frc.robot.subsystems.Drivetrain;

import java.util.Set;

public class VisionTrack implements Command {

    private static final PIDController TURN_PID_CONTROLLER = new PIDController(VisionConstants.kPTurn,
            VisionConstants.kITurn, VisionConstants.kDTurn);
    private static final PIDController DIST_PID_CONTROLLER = new PIDController(VisionConstants.kPDist,
            VisionConstants.kIDist, VisionConstants.kDDist);

    private Subsystem[] requirements = { RobotContainer.drivetrain };

    public VisionTrack() {

    }

    @Override
    public void initialize() {
        RobotContainer.getInstance().setLEDMode(LEDMode.ON);
        RobotContainer.getInstance().setPipeline(VisionPipeline.VISION);

    }

    @Override
    public void execute() {

        double left, right;

        double turnError = RobotContainer.getXOffset();
        double distError = RobotContainer.getYOffset();

        if (turnError < VisionConstants.kTurnTolerance) turnError = 0;
        if (distError < VisionConstants.kDistTolerance) distError = 0;

        double throttle = DIST_PID_CONTROLLER.calculate(distError, 0);
        double turn = TURN_PID_CONTROLLER.calculate(turnError, 0);

        if (throttle != 0) {
            throttle *= DrivetrainConstants.kMaxSpeedMPS;
            turn *= DriverConstants.kMaxCurvature * throttle;

            DifferentialDriveWheelSpeeds wSpeeds = Drivetrain.KINEMATICS.toWheelSpeeds(new ChassisSpeeds(throttle, 0, turn));
            wSpeeds.normalize(DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kDriveSens);

            left = wSpeeds.leftMetersPerSecond / DrivetrainConstants.kMaxSpeedMPS + Drivetrain.FEEDFORWARD.calculate(wSpeeds.leftMetersPerSecond) / Constants.kMaxVoltage;
            right = wSpeeds.rightMetersPerSecond / DrivetrainConstants.kMaxSpeedMPS + Drivetrain.FEEDFORWARD.calculate(wSpeeds.rightMetersPerSecond) / Constants.kMaxVoltage;

        } else {
            // Turns in place when there is no throttle input
            left = turn * DriverConstants.kTurnInPlaceSens;
            right = -turn * DriverConstants.kTurnInPlaceSens;
        }

        Drivetrain.setOpenLoop(left, right);

    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.getInstance().setLEDMode(LEDMode.OFF);
        Drivetrain.setOpenLoop(0.0, 0.0);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
