package frc.robot;

import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OrchestraConstants;
import frc.robot.autonomous.Dashboard;
import frc.robot.autonomous.TrajectoryTracker;
import frc.robot.commands.ConveyorQueue;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

public class RobotContainer {
    public static Drivetrain drivetrain;
    public static Orchestra orchestra;
    
    private static int songIndex = 0;
    
    public static Intake intake;
    public static Conveyor conveyor;
    public static Shooter shooter;

    public static Dashboard falconDashboard;

    public static AHRS navX;
    
    private static final XboxController driver = new XboxController(Constants.InputPorts.xboxController);
    private static final JoystickButton driver_X = new JoystickButton(driver, 3);
    private static final POVButton DPAD_RIGHT = new POVButton(driver, 90);
    private static final POVButton DPAD_LEFT = new POVButton(driver, 270);
    
    private static RobotContainer instance;
    public static RobotContainer getInstance(){
        if (instance == null) instance = new RobotContainer();
        return instance;
    }
    
    private RobotContainer(){
        navX = new AHRS(Port.kMXP);
        
        drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new Drive(Drive.State.CheesyDriveOpenLoop));
        
        orchestra = new Orchestra(List.of(Drivetrain.motors));
        // remove to unbind orchestra OI
        bindOrchestraOI();
    
        intake = Intake.getInstance();
    
        conveyor = Conveyor.getInstance();
        conveyor.setDefaultCommand(new ConveyorQueue());
    
        shooter = Shooter.getInstance();
    
        falconDashboard = Dashboard.getInstance();
    
        bindOI();
    }
    
    private void bindOrchestraOI(){
        orchestra.loadMusic(OrchestraConstants.songs[0]);
        driver_X.whenPressed(() -> orchestra.play(), drivetrain).whenReleased(() -> orchestra.pause());
        DPAD_RIGHT.whenPressed(() -> {
            songIndex++;
            if (songIndex > OrchestraConstants.numSongs) songIndex = 0;
            orchestra.loadMusic(OrchestraConstants.songs[songIndex]);
        });
        DPAD_LEFT.whenPressed(() -> {
            songIndex--;
            if (songIndex < 0) songIndex = OrchestraConstants.numSongs;
            orchestra.loadMusic(OrchestraConstants.songs[songIndex]);
        });
    }

    /* Binding OI input to Commands */
    private void bindOI() {

    }

    public Command getAutonomousCommand() {
        // TODO: Replace this code with the selector logic to select the proper autonomousit  sequence (aka, decide how we want to select auto)
        Trajectory initialTrajectory;
        try {
            initialTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/InitiationToTarget.json"));
        } catch(IOException e) {
            initialTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 5, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 5.5),
                new Translation2d(2, 4.5)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 5, new Rotation2d(0)),
            // Pass config
            new TrajectoryConfig(2, 4)
                .setKinematics(Drivetrain.KINEMATICS)
                .addConstraint(new DifferentialDriveVoltageConstraint(Drivetrain.FEEDFORWARD, Drivetrain.KINEMATICS, 10))
            );
        }

        // Workaround for try-catch block "initialTrajectory may have already been assigned"
        final Trajectory initialAutonomousTrajectory = initialTrajectory;

        /* Commands common to all autonomous sequences */
        SequentialCommandGroup initializeAutonomous = new SequentialCommandGroup(
            new InstantCommand(navX::reset),
            new InstantCommand(() -> Drivetrain.ODOMETRY.resetPosition(initialAutonomousTrajectory.sample(0).poseMeters, 
                                     Rotation2d.fromDegrees(navX.getAngle()))),
            new InstantCommand(drivetrain::resetEncoders)
        );

        /* Adding all the autonomous commands into a single sequence */
        return new SequentialCommandGroup(initializeAutonomous, new TrajectoryTracker(initialTrajectory));
    }
    
    public static double getThrottleValue() {
        // Controllers y-axes are natively up-negative, down-positive. This method corrects that by returning the opposite of the y-value
        return -deadbandX(driver.getY(GenericHID.Hand.kLeft), DriverConstants.kJoystickDeadband);
    }
    
    public static double getTurnValue() {
        return deadbandX(driver.getX(GenericHID.Hand.kRight), DriverConstants.kJoystickDeadband);
    }
    
    public static double deadbandX(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0;
        } else if (Math.abs(input) == 1) {
            return input;
        } else {
            return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
        }
    }
}
