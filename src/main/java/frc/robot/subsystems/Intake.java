/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Intake implements Subsystem {
  private final TalonSRX intakeSRX = new TalonSRX(8);
  private final ArmFeedforward feedforward = new ArmFeedforward(IntakeConstants.kSVolts, 
  IntakeConstants.kCosVolts, IntakeConstants.kVVoltSecondPerRad, 
  IntakeConstants.kAVoltSecondSquaredPerRad);
  /**
   * Creates a new Intake.
   */

  private static Intake instance = null;
  public static Intake getInstance(){
    if(instance == null){
      instance = new Intake();
    }
    return instance;
  }
  public Intake() {
    intakeSRX.configPeakCurrentLimit(45);
    intakeSRX.configPeakCurrentDuration(125);
    intakeSRX.configContinuousCurrentLimit(38);
    intakeSRX.enableCurrentLimit(true);

    ProfiledPIDSubsystem pidSubsystem = new ProfiledPIDSubsystem(new ProfiledPIDController(IntakeConstants.kP, 0, 0, new TrapezoidProfile.Constraints(
      ArmConstants.kMaxVelocityRadPerSecond, ArmConstants.kMaxAccelerationRadPerSecSquared)), 0);
  m_encoder.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse));
  // Start arm at rest in neutral position
  setGoal(ArmConstants.kArmOffsetRads);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motor.setVoltage(output + feedforward);
  }

  public double getMeasurement() {
    return m_encoder.getDistance() + IntakeConstants.kArmOffsetRads;
  }
}
