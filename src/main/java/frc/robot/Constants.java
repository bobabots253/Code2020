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
        
        public static final double kTurnSens = 1; // Maximum normal turning rate (in percent of max) to allow robot to turn to [0,1]
        public static final double kDriveSens = 1; // Overall speed setting (turn down for demos) [0,1]
        public static final double kTurnInPlaceSens = 0.25; // Maximum turn-in-place rate (in percent of max) to allow robot to turn to [0,1]
    }
    
    
    public static class DrivetrainConstants {
        public static final int
            /* Drivetrain motor IDs */
            leftMaster = 0,   // TalonFX
            leftSlave = 1,    // TalonFX
            rightMaster = 2,  // TalonFX
            rightSlave = 3;   // TalonFX

        public static final double kMaxSpeedMPS = 4.972;  // max speed in meters per second
        public static final double kMaxCurvature = Math.toRadians(-180);  // max turn rate in radians per second
        public static final double trackWidth = 0.7051868402911773;  // distance between wheels

        /* feedforward constants */
        public static final double kS = 0.364;  // voltage required to overcome friction (V)
        public static final double kV = 2.34;  // voltage over velocity                 (V/(meters/second))
        public static final double kA = 0.0824;  // voltage over racceleration            (V(meters/second/second))

        /* PID constants */
        public static final double kP = 2.99;
        public static final double kI = 0;
        public static final double kD = 0;

        /*  */
        public static final double ticksPerRotation = 2048 * 10.42;
        public static final double wheelDiameter = 6 / 39.3700787;  // first number inches --> converted to meters

        /* Drivetrain Current Limiting */
        public static final boolean kLimitEnabled = true;  // never turn this to false
        // TODO: Tune these
        public static final double kCurrentLimit = 45;
        public static final double kTriggerThresholdCurrent = 38;
        public static final double kTriggerThresholdTimeDelta = 0.125;
    }
}
