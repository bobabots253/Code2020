package frc.robot;


import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OrchestraConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

import java.util.Arrays;

public class RobotContainer {
    public static Drivetrain drivetrain;
    public static Orchestra orchestra;
    
    private static int songIndex = 0;
    
    private static RobotContainer instance;
    private static final XboxController driver = new XboxController(Constants.InputPorts.xboxController);
    private static final JoystickButton driver_X = new JoystickButton(driver, 3);
    private static final POVButton DPAD_RIGHT = new POVButton(driver, 90);
    private static final POVButton DPAD_LEFT = new POVButton(driver, 270);
    
    public static RobotContainer getInstance(){
        if (instance == null) instance = new RobotContainer();
        return instance;
    }
    
    private RobotContainer(){
        drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new Drive(Drive.State.CheesyDriveOpenLoop));
        
        orchestra = new Orchestra(Arrays.asList(Drivetrain.motors));
        // remove to unbind orchestra OI
        bindOrchestraOI();
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
    
    public static double getThrottleValue() {
        // Controllers y-axes are natively up-negative, down-positive. This method corrects that by returning the opposite of the value
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
