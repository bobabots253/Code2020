package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.Subsystem.*;
import frc.robot.Command.*;

public class RobotContainer {
    public static Shooter shooter;
    private static RobotContainer instance;
    private static final XboxController driver = new XboxController(Constants.InputPorts.xboxController);
    private static final Joystick operator = new Joystick(1);//used now
    private static final JoystickButton button1 = new JoystickButton(operator, 1);
    
    public static RobotContainer getInstance(){
        if (instance == null) instance = new RobotContainer();
        return instance;
    }
    
    private RobotContainer(){
        shooter = Shooter.getInstance();
        bindOI();
    }

    private void bindOI(){
        button1.whenPressed(new Shoot());
    }

    public static double getThrottleValue() {
        // Controllers y-axes are natively up-negative, down-positive. This method corrects that by returning the opposite of the value
        return -deadbandX(driver.getY(GenericHID.Hand.kLeft), DriverConstants.kJoystickDeadband);
    }
    
    public static double getJoystickYValue(){
        return operator.getY();
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