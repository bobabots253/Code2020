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
        public static final double kTriggerDeadband = 0.05;  // How much of trigger  is "dead" zone [0,1]
        
        public static final double kTurnSens = 1; // Maximum normal turning rate (in percent of max) to allow robot to turn to [0,1]
        public static final double kDriveSens = 1; // Overall speed setting (turn down for demos) [0,1]
        public static final double kTurnInPlaceSens = 1; // Maximum turn-in-place rate (in percent of max) to allow robot to turn to [0,1]
        
        public static final double kMaxTurnRateRadians = Math.PI/3.0; // Maximum radians per second to allow turning
    }
    
    
    public static class DriveTrainConstants {
        public static final int
            /* Drivetrain motor IDs */
            leftMaster = 0,   // TalonFX
            leftSlave = 1,    // TalonFX
            rightMaster = 2,  // TalonFX
            rightSlave = 3;   // TalonFX
    }
}
