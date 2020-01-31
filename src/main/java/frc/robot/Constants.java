package frc.robot;

public class Constants {
    /* */
    public static class InputPorts {
        public static final int
            xboxController = 0;
    }
    
    /* Differential Drive Settings */
    public static class DriverConstants {
        // copied from 2019
        public static final double kJoystickDeadband = 0.07; // How much of joystick is "dead" zone [0,1]
        
        public static final double kTurnSens = 0.75; // Maximum normal turning rate (in percent of max) to allow robot to turn to [0,1]
        public static final double kDriveSens = 0.75; // Overall speed setting (turn down for demos) [0,1]
        public static final double kTurnInPlaceSens = 0.75; // Maximum turn-in-place rate (in percent of max) to allow robot to turn to [0,1]
    }
    
    
    public static class DriveTrainConstants {
        public static final int
            /* Drivetrain motor IDs */
            leftMaster = 0,   // TalonFX
            leftSlave = 1,    // TalonFX
            rightMaster = 2,  // TalonFX
            rightSlave = 3;   // TalonFX

        /* Drivetrain Current Limiting */
        public static final boolean kLimitEnabled = true;  // never turn this to false
        // TODO: Tune these
        public static final double kCurrentLimit = 45;
        public static final double kTriggerThresholdCurrent = 38;
        public static final double kTriggerThresholdTimeDelta = 0.125;
    }

    public static class ShooterConstants{
        public static final int shooterMotor = 1;  // neo ids
    }
}