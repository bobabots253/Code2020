package frc.robot.autonomous;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;

// Interfaces with FalconDashboard to visualize path odometry
// https://github.com/5190GreenHopeRobotics/FalconDashboard

public class Dashboard {
    private NetworkTableEntry robotX;
    private NetworkTableEntry robotY;
    private NetworkTableEntry robotHeading;

    private NetworkTableEntry pathX;
    private NetworkTableEntry pathY;
    private NetworkTableEntry pathHeading;

    private NetworkTableEntry followingPath;

    private static Dashboard instance;
    public static Dashboard getInstance() {
        if(instance == null) instance = new Dashboard();
        return instance;
    }

    private Dashboard(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Live_Dashboard");

        robotX = table.getEntry("robotX");
        robotY = table.getEntry("robotY");
        robotHeading = table.getEntry("robotHeading");

        pathX = table.getEntry("pathX");
        pathY = table.getEntry("pathY");
        pathHeading = table.getEntry("pathHeading");

        followingPath = table.getEntry("isFollowingPath");
        this.followingPath.setBoolean(false);
        
    }

    /**
     * Puts drivetrain pose data into the FalconDashboard NetworkTable
     * @param pose The current pose of the robot
     */
    public void putOdom(Pose2d pose){
        this.robotX.setDouble(pose.getTranslation().getX());
        this.robotY.setDouble(pose.getTranslation().getY());
        this.robotHeading.setDouble(pose.getRotation().getRadians());
    }
    
    /**
     * Puts path pose data into the FalconDashboard NetworkTable
     * @param pose The current pose of the path
     */
    public void putPath(Pose2d pose){
        this.pathX.setDouble(pose.getTranslation().getX());
        this.pathY.setDouble(pose.getTranslation().getY());
        this.pathHeading.setDouble(pose.getRotation().getRadians());
        this.followingPath.setBoolean(true);
    }

    /**
     * Sets the "followingPath" entry of the NetworkTable to false, indicating that path odometry should stop 
     */
    public void endPath(){
        this.followingPath.setBoolean(false);
    }
}