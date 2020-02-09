package frc.robot;

public class Constants {
    /* Controllers Input */
    public static class InputPorts {
        public static final int
            xboxController = 0;
    }
    
    /* Differential Drive Settings */
    public static class DriverConstants {
        /* Common drive mode settings */
        public static final double kJoystickDeadband = 0.07; // How much of joystick is "dead" zone [0,1]
        public static final double kDriveSens = 1; // Overall speed setting (turn down for demos) [0,1]
        public static final double kTurnInPlaceSens = 0.25; // Maximum turn-in-place rate (in percent of max) to allow robot to turn to [0,1]
        
        /* 2019 drive mode constants */
        public static final double kTurnSens = 1; // Maximum normal turning rate (in percent of max) to allow robot to turn to [0,1]

        /* Cheesydrive constants */
        public static final double kMaxCurvature = Math.toRadians(-180);  // Maximum turn rate in radians per meter [0, infinity]
    }
    
    
    public static class DrivetrainConstants {
        public static final int
            /* Drivetrain motor IDs */
            leftMaster = 0,   // TalonFX
            leftSlave = 1,    // TalonFX
            rightMaster = 2,  // TalonFX
            rightSlave = 3;   // TalonFX

        /* feedforward constants */
        public static final double kS = 0.364;  // voltage required to overcome friction (V)
        public static final double kV = 2.34;  // voltage over velocity                  (V/(meters/second))
        public static final double kA = 0.0824;  // voltage over acceleration            (V(meters/second/second))

        /* PID constants */
        public static final double kP = 2.99;
        public static final double kI = 0;
        public static final double kD = 0;

        /* Wheels Constants */
        public static final double kTicksPerRotation = 2048 * 10.42;
        public static final double kWheelDiameter = 6 / 39.3700787;  // first number inches --> converted to meters
        
        public static final double kMaxSpeedMPS = 4.972;  // max speed in meters per second
        public static final double kTrackWidth = 0.7051868402911773;  // distance between wheels
    
        /* Drivetrain Current Limiting */
        public static final boolean kLimitEnabled = true;  // never turn this to false
        // TODO: Tune these
        public static final double kCurrentLimit = 45;
        public static final double kTriggerThresholdCurrent = 38;
        public static final double kTriggerThresholdTimeDelta = 0.125;

        public static final double kRamseteBeta = 2.0;
        public static final double kRamseteZeta = 0.7;
    }
    
    public static class IntakeConstants {
        /* Motors */ // TODO: Change these numbers
        public static final int armMotor = 8;
        public static final int spinMotor = 9;
        
        /* PID Constants */ //TODO: Tune
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;

        /* Feedforward Constants */ // TODO: tune
        public static double kS = 0;
        public static double kCos = 0;
        public static double kV = 0;
        public static double kA = 0;

        /* Intake constants */ // TODO: tune
        public static double kMaxVelocity = 0; // Maximum velocity to turn arm at, radians per second
        public static double kMaxAcceleration = 0; // Maximum acceleration to turn arm at, radians per second per second
        public static double kRadPerTick = 0; // Angle change of the arm per tick of the encoder
    }
    
    public static class ConveyorConstants {
        //TODO: Change this number
        public static final int motorID = 10;

        public static final double kQueueSpeed = 0.1;
    }
    
    public static class ShooterConstants {
        //TODO: Change this number
        public static final int motorID = 11;

        /* PID Constants */ //TODO: Tune
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;

        /* Feedforward Constants */ // TODO: tune
        public static double kS = 0;
        public static double kV = 0;
        public static double kA = 0;

        /* Shooter Constants */
        public static double kTolerance = 0;
    }
}

