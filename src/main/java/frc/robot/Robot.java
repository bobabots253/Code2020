/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  RobotContainer robot;
  Command autonomous;


  @Override
  public void robotInit() {
    robot = RobotContainer.getInstance();
    RobotContainer.drivetrain.resetEncoders();
    RobotContainer.navX.reset();
    RobotContainer.intake.resetEncoders();

  }
  
  @Override
  public void disabledInit(){
  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("dt left enc", Drivetrain.getLeftEnc());
    SmartDashboard.putNumber("dt right enc", Drivetrain.getRightEnc());
  }

  @Override
  public void autonomousInit() {
    autonomous = RobotContainer.getInstance().getAutonomousCommand();
    if (autonomous != null) autonomous.schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }


  @Override
  public void teleopPeriodic() {
    if (autonomous != null) autonomous.cancel();
  }

  @Override
  public void testPeriodic() {
  }
}
