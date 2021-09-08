package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class Climb extends WaitCommand {

    public enum Side {
        LEFT, RIGHT, BOTH, DOWN, RIGHTDOWN, LEFTDOWN
    }
    public static Timer timer;
    public static double startTime;
    public static double newStartTime;
    Side side;
    double speed;
    double initSpeed;
    private Climber climber = RobotContainer.climber;
    
    public Climb(Side side, double speed) {
        super(0.2);
        timer = new Timer();
        startTime = 0;
        this.side = side;
        this.speed = speed;
        
        m_requirements.add(climber);
    }
    
    @Override
    public void initialize() {
        switch (side) {
            case LEFT:
                break;
            case BOTH:
            case DOWN:
            case RIGHT:
                break;
            case RIGHTDOWN:
            case LEFTDOWN:
                break;
        }
        
        super.initialize();
    }
    
    @Override
    public void execute() {
        if (startTime == 0) {startTime = timer.get(); timer.start();}
        //double tempSpeed = speed;
        // only runs if the time has elapsed
        if (super.isFinished()) {
            switch (side) {
                case LEFT:
                    //running left motor down
                    // if (timer.get() - startTime <= 0.35) {
                    //     climber.climbLeft(-speed);
                    //     System.out.println("left motor going down");
                    // } else if (timer.get() - startTime > 0.35 && timer.get() - startTime <= 3) {
                    //     climber.stopLeftMotor();
                    //     climber.leftServoDown();
                    //     System.out.println("left motor paused");
                    // } else if (timer.get() - startTime > 3) {
                    //     climber.climbLeft(speed);
                    //     System.out.println("left motor going up");
                    // }

                    climber.climbLeft(-speed);
                    break;
                case RIGHT:
                    //running right motor down
                //     if (timer.get() - startTime <= 0.35) {
                //     climber.climbRight(-speed);
                //     System.out.println("right motor going down");
                // } else if (timer.get() - startTime > 0.35 && timer.get() - startTime <= 3) {
                //     climber.stopRightMotor();
                //     climber.rightServoDown();
                //     System.out.println("right motor paused");
                // } else if (timer.get() - startTime > 3) {
                //     climber.climbRight(speed);
                //     System.out.println("right motor going up");
                // }

                    climber.climbRight(-speed);
                    //climber.rightServoDown();
                    break;
                
                case BOTH:    
                    //running motors down
                    // if (timer.get() - startTime <= 0.35) {
                    //     climber.climbUnity(-speed);
                    //     System.out.println("climber oppostie way");
                    // } else if (timer.get() - startTime > 0.35 && timer.get() - startTime <= 3) {
                    //     //delay motors and put servos down
                    //     climber.stopMotors();
                    //     climber.leftServoDown();
                    //     climber.rightServoDown();
                    //     System.out.println("climber paused");
                    // } else if (timer.get() - startTime > 3) {
                    //     climber.climbUnity(speed);
                    //     System.out.println("climber running");
                    // }

                    climber.climbUnity(-speed); //-speed, bring arms up
                    break;
                case DOWN:
                        // climber.servosDown();
                        // climber.climbUnity(-speed);
                        if (timer.get() - startTime <= 0.35) {
                            climber.climbUnity(-speed);
                            System.out.println("climber opposite way");
                        } 
                        else if (timer.get() - startTime > 0.35 && timer.get() - startTime <= 3) {
                            //delay motors and put servos down
                            climber.stopMotors();
                            climber.leftServoDown();
                            climber.rightServoDown();
                            System.out.println("climber paused");
                        } else if (timer.get() - startTime > 3) {
                            climber.leftServoDown();
                            climber.rightServoDown();
                            climber.climbUnity(speed);
                            System.out.println("climber running");

                        }
                        break;
                case RIGHTDOWN:

                if(newStartTime == 0) {newStartTime = timer.get();}
                       //running right motor down
                    if (timer.get() - newStartTime <= 0.35) {
                        climber.climbRight(-speed);
                        System.out.println("right motor going up");
                    } else if (timer.get() - newStartTime > 0.35 && timer.get() - newStartTime <= 3) {
                        climber.stopRightMotor();
                        climber.rightServoDown();
                        System.out.println("right motor paused");
                    } else if (timer.get() - newStartTime > 3) {
                        climber.climbRight(speed);
                        System.out.println("right motor going down");
                    }
                    break;
                case LEFTDOWN:
                    newStartTime = 0;
                    if(newStartTime == 0) {newStartTime = timer.get();}
                    //running left motor down
                    
                    if (timer.get() - newStartTime <= 0.35) {
                        climber.climbLeft(-speed);
                        System.out.println("left motor going up");
                    } else if (timer.get() - newStartTime > 0.35 && timer.get() - newStartTime <= 3) {
                        climber.stopLeftMotor();
                        climber.leftServoDown();
                        System.out.println("left motor paused");
                    } else if (timer.get() - newStartTime > 3) {
                        climber.climbLeft(speed);
                        System.out.println("left motor going down");
                    }
                    break;
               default:
                    break;

            }
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
       
        if (!interrupted) {
            switch (side) {
                case LEFT:
                    climber.stopLeftMotor();
                    break;
                case BOTH:
                    climber.stopLeftMotor();
                case RIGHT:
                    climber.stopRightMotor();
                    break;
            }
        }
        super.end(interrupted);
    }
}
