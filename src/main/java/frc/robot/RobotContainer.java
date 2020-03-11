
package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.autonomous.Dashboard;
import frc.robot.autonomous.TrajectoryTracker;
import frc.robot.commands.ConveyorQueue;
import frc.robot.commands.Shoot;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    public static Drivetrain drivetrain;
    public static Intake intake;
    public static Conveyor conveyor;
    public static Shooter shooter;
    public static Climber climber;
    public static Arm arm;
    public boolean goShooter = false;
    public static Dashboard falconDashboard;
    private static NetworkTable limelight;
    public static AHRS navX;

    private static final XboxController driver = new XboxController(Constants.InputPorts.driver_Controller);

    private static final XboxController operator = new XboxController(Constants.InputPorts.operator_Controller);

    private static final JoystickButton driver_A = new JoystickButton(driver, 1),
            driver_B = new JoystickButton(driver, 2), driver_X = new JoystickButton(driver, 3),
            driver_Y = new JoystickButton(driver, 4), driver_LB = new JoystickButton(driver, 5),
            driver_RB = new JoystickButton(driver, 6), driver_VIEW = new JoystickButton(driver, 7),
            driver_MENU = new JoystickButton(driver, 8);
    private static final JoystickButton operator_A = new JoystickButton(operator, 1),
            operator_B = new JoystickButton(operator, 2), operator_X = new JoystickButton(operator, 3),
            operator_Y = new JoystickButton(operator, 4), operator_LB = new JoystickButton(operator, 5),
            operator_RB = new JoystickButton(operator, 6), operator_VIEW = new JoystickButton(operator, 7),
            operator_MENU = new JoystickButton(operator, 8);
        
    private static final POVButton driver_DPAD_UP = new POVButton(driver, 0),
            driver_DPAD_RIGHT = new POVButton(driver, 90), driver_DPAD_DOWN = new POVButton(driver, 180),
            driver_DPAD_LEFT = new POVButton(driver, 270);

    private static final POVButton operator_DPAD_UP = new POVButton(operator, 0),
            operator_DPAD_RIGHT = new POVButton(driver, 90), operator_DPAD_DOWN = new POVButton(driver, 180),
            operator_DPAD_LEFT = new POVButton(driver, 270);

    private static RobotContainer instance;

    public static RobotContainer getInstance() {
        if (instance == null)
            instance = new RobotContainer();
        return instance;
    }

    private RobotContainer() {
        navX = new AHRS(Port.kMXP);
        limelight = NetworkTableInstance.getDefault().getTable("limelight");

        drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new Drive(Drive.State.CheesyDriveOpenLoop));

        intake = Intake.getInstance();

        conveyor = Conveyor.getInstance();
        conveyor.setDefaultCommand(new ConveyorQueue(ConveyorQueue.State.None));

        climber = Climber.getInstance();

        arm = Arm.getInstance();

        shooter = Shooter.getInstance();

        falconDashboard = Dashboard.getInstance();

        bindOI();
    }

    /**
     * Binds operator input to Commands 
     */
    private void bindOI() {
        
       // Flip down intake arm and spin when RB is held, flip back up and stop spinning when released
        driver_RB.whileHeld(new RunCommand(()->arm.rotate(-0.4), arm)
                    .alongWith(new RunCommand( ()->intake.intake(0.5)))
                    .alongWith(new RunCommand( ()->intake.setConveyor(0.5))))
                .whenReleased(new RunCommand( ()->arm.rotate(0.35), arm)
                    .alongWith(new InstantCommand(intake::stopIntake)));
    
        // Spin up shooter when LB is held, stop when released
        driver_LB.whileHeld(new Shoot());

        // Flip intake down and spin outwards to sweep balls out of the way when A is held, flip up and stop when released
        driver_A.whileHeld(new RunCommand(()->arm.rotate(-0.4), arm)
                    .alongWith(new RunCommand( ()->intake.intake(-0.5))))
                .whenReleased(new RunCommand( ()->arm.rotate(0.35), arm)
                    .alongWith(new InstantCommand(intake::stopIntake)));   

        // Run both climbers when DPAD up is heldgit 
        operator_DPAD_UP.whileHeld(new RunCommand(() -> climber.climbUnity(0.5), climber)).whenReleased(new InstantCommand(climber::stopMotors, climber)); 
        //operator_DPAD_DOWN.whileHeld(new RunCommand(() -> climber.climbUnity(-0.5), climber)).whenReleased(new InstantCommand(climber::stopMotors, climber)); 

        // Run the right and left climbers when view and menu are held, respectively
        operator_VIEW.whileHeld(new RunCommand(() -> climber.climbLeft(0.5), climber)).whenReleased(new RunCommand(climber::stopLeftMotor, climber));
        operator_MENU.whileHeld( new RunCommand(() -> climber.climbRight(0.5), climber)).whenReleased(new RunCommand(climber::stopRightMotor, climber));

        // 2nd Controller vertical conveyor up and down respectively
        operator_Y.whileHeld(new RunCommand(()-> conveyor.setOpenLoop(0.55), conveyor))
                   .whenReleased(new InstantCommand(conveyor::stop, conveyor));
        operator_A.whileHeld(new RunCommand(()-> conveyor.setOpenLoop(-0.55), conveyor)) 
                   .whenReleased(new InstantCommand(conveyor::stop, conveyor));

        // 2nd controller horizontal conveyor in and out respectively
        operator_X.whileHeld(new RunCommand(()-> intake.setConveyor(0.5), intake))
                     .whenReleased(new InstantCommand(intake::stopIntake, intake));
        operator_B.whileHeld(new RunCommand(()-> intake.setConveyor(-0.5), intake))
        .whenReleased(new InstantCommand(intake::stopIntake, intake));
        
    }

    /**
     * Constructs and returns the Command to run during the autonomous period
     * 
     * @return the Command to run during autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new StartEndCommand(
                ()-> Drivetrain.setOpenLoop(0.2,0.2),
                Drivetrain::stop,
                drivetrain
            ).withTimeout(3), 
            new WaitCommand(1),
            new Shoot().withTimeout(3));
    }

    public Command getAutonomousCommand(int driveDelay, int shootDelay) {
        return new SequentialCommandGroup(
            new WaitCommand(driveDelay),
            new StartEndCommand(
                ()-> Drivetrain.setOpenLoop(0.2,0.2),
                Drivetrain::stop,
                drivetrain
            ).withTimeout(3), 
            new WaitCommand(shootDelay),
            new Shoot().withTimeout(3));
    }

    public Command getTrenchAutonomousCommand(int driveDelay, int shootDelay) {
        return new SequentialCommandGroup(
            new WaitCommand(driveDelay),
            new StartEndCommand(
                ()-> Drivetrain.setOpenLoop(0.2,0.2),
                Drivetrain::stop,
                drivetrain
            ).withTimeout(3), 
            new WaitCommand(shootDelay),
            new RunCommand(()->arm.rotate(-0.4), arm)
            .alongWith(new RunCommand( ()->intake.intake(0.5)))
            .alongWith(new RunCommand( ()->intake.setConveyor(0.5))),
            new StartEndCommand(
                ()-> Drivetrain.setOpenLoop(0.2,0.2),
                Drivetrain::stop,
                drivetrain
            ).withTimeout(3), 
            new StartEndCommand(
                ()-> Drivetrain.setOpenLoop(-0.2,0.2),
                Drivetrain::stop,
                drivetrain
            ).withTimeout(3),
            new StartEndCommand(
                ()-> Drivetrain.setOpenLoop(0.2,0.2),
                Drivetrain::stop,
                drivetrain
            ).withTimeout(3), 
            new Shoot().withTimeout(3));
    }



    /**
     * Returns the deadbanded throttle input from the main driver controller
     * 
     * @return the deadbanded throttle input from the main driver controller
     */
    public static double getThrottleValue() {
        // Controllers y-axes are natively up-negative, down-positive. This method
        // corrects that by returning the opposite of the y-value
        return -deadbandX(driver.getY(GenericHID.Hand.kLeft), DriverConstants.kJoystickDeadband);
    }

    /**
     * Returns the deadbanded turn input from the main driver controller
     * 
     * @return the deadbanded turn input from the main driver controller
     */
    public static double getTurnValue() {
        return deadbandX(driver.getX(GenericHID.Hand.kRight), DriverConstants.kJoystickDeadband);
    }

    /**
     * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be
     * linear from (deadband, 0) to (1,1)
     * 
     * @param input    The input value to rescale
     * @param deadband The deadband
     * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
     */
    public static double deadbandX(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0;
        } else if (Math.abs(input) == 1) {
            return input;
        } else {
            return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
        }
    }

    /**
     * Set the LED mode on the Limelight
     * 
     * @param ledMode The mode to set the Limelight LEDs to
     */
    public void setLEDMode(LEDMode ledMode) {
        limelight.getEntry("ledMode").setNumber(ledMode.val);
    }

    /**
     * Sets the appearance of the Limelight camera stream
     * 
     * @param stream Stream mode to set the Limelight to
     */
    public void setStreamMode(StreamMode stream) {
        limelight.getEntry("stream").setNumber(stream.val);
    }

    /**
     * Sets Limelight vision pipeline
     * 
     * @param pipeline The pipeline to use
     */
    public void setPipeline(VisionPipeline pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline.val);
    }

    /**
     * Returns the horizontal offset between the target and the crosshair in degrees
     * 
     * @return the horizontal offset between the target and the crosshair in degrees
     */
    public static double getXOffset() {
        return -limelight.getEntry("tx").getDouble(0);
    }

    /**
     * Returns the vertical offset between the target and the crosshair in degrees
     * 
     * @return the vertical offset between the target and the crosshair in degrees
     */
    public static double getYOffset() {
        return -limelight.getEntry("ty").getDouble(0.0);
    }

    /**
     * Enum representing the different possible Limelight LED modes
     */
    public enum LEDMode {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        public int val;

        LEDMode(int val) {
            this.val = val;
        }
    }

    /**
     * Enum representing the different possible Limelight stream modes
     */
    public enum StreamMode {
        SIDE_BY_SIDE(0), PIP_MAIN(1), PIP_SECONDARY(2);

        public int val;

        StreamMode(int val) {
            this.val = val;
        }
    }

    /**
     * Enum representing the different possible Limelight vision pipelines
     */
    public enum VisionPipeline {
        VISION(0), DRIVER(1);

        public int val;

        VisionPipeline(int val) {
            this.val = val;
        }
    }

}
