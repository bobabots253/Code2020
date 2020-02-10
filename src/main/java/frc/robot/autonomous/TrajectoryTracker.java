package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.Units;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryTracker extends RamseteCommand {
    private Trajectory trajectory;
    private double startTime;

    /**
     * This class follows a given Trajectory using RamseteCommand, providing odometry functionality
     * 
     * @param trajectory The Trajectory to follow
     */
    public TrajectoryTracker(Trajectory trajectory) {
        super(trajectory, 
              Drivetrain.ODOMETRY::getPoseMeters,
              new RamseteController(DrivetrainConstants.kRamseteBeta, DrivetrainConstants.kRamseteZeta),
              Drivetrain.FEEDFORWARD,
              Drivetrain.KINEMATICS,
              Drivetrain::getWheelSpeeds,
              Drivetrain.LEFT_PID_CONTROLLER,
              Drivetrain.RIGHT_PID_CONTROLLER,
              Drivetrain::setOpenLoop,
              RobotContainer.drivetrain);
        this.trajectory = trajectory;
    }
    
    @Override
    public void initialize(){
        super.initialize();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        super.execute();
        double currentTime = Timer.getFPGATimestamp();

        Trajectory.State currentState = trajectory.sample(currentTime-startTime);
        Pose2d currentPose = currentState.poseMeters;

        RobotContainer.falconDashboard.putPath(Units.MeterPoseToFeetPose(currentPose));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Drivetrain.stopMotors();
        RobotContainer.falconDashboard.endPath();
    }
}