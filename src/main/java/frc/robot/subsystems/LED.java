package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LEDConstants;
 
public class LED implements Subsystem{
    
    CANifier canifier = new CANifier(LEDConstants.canifierID);
    private static LED instance;
    public static LEDConstants.State state = LEDConstants.State.NULL;
    public static Color color;
    public static double startONTime = 0.0;
    public static double startOFFTime = 0.0;
    private LED(){
        register();
    }  

    @Override 
    public void periodic(){
        switch(state){
            case SHOOTING: { setColor(Color.kBlack, 0.0); break; }
            case INTAKE: { setColor(Color.kBlue, 0.2); break; }
            case ENABLED: { setColor(Color.kDarkTurquoise, 0.0); break; } 
            case DISABLED: { setColor(Color.kGreen, 0.0); break; }
            case ERROR: { setColor(Color.kRed, 0.0); break; }
            case NULL: { setRGB(0,0,0,0.0); startONTime = 0; break;}
            default: { setColor(Color.kBlue, 0.0); break; } //just set color to blue if we cant figure out y the wrong colors are being displayed
        }
    }

    public static LED getInstance(){
        if(instance == null){instance = new LED();}
        return instance;
    }
    public void setRGB(double red, double green, double blue, double blinkTiming){
        if(blinkTiming == 0.0){
            canifier.setLEDOutput(green, LEDChannel.LEDChannelA);
            canifier.setLEDOutput(red, LEDChannel.LEDChannelB);
            canifier.setLEDOutput(blue, LEDChannel.LEDChannelC);
        }
        else{
        if(startONTime == 0.0)   {startONTime = Timer.getFPGATimestamp();}
        if((Timer.getFPGATimestamp() - startONTime ) < blinkTiming){
            canifier.setLEDOutput(green, LEDChannel.LEDChannelA);
            canifier.setLEDOutput(red, LEDChannel.LEDChannelB);
            canifier.setLEDOutput(blue, LEDChannel.LEDChannelC);
        }
        else {
            if(startOFFTime == 0.0) 
                startOFFTime = Timer.getFPGATimestamp();
            if((Timer.getFPGATimestamp() - startOFFTime) >= LEDConstants.blinkTime){ //resets to light
                startONTime = Timer.getFPGATimestamp();
                startOFFTime = 0.0;
            }
            canifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
            canifier.setLEDOutput(0.0, LEDChannel.LEDChannelB);
            canifier.setLEDOutput(0.0, LEDChannel.LEDChannelC);
        }
    }
    }

    public void setColor(Color color, double blinkTiming){
        setRGB(color.red, color.green, color.blue, blinkTiming);
    }
    public void setState(LEDConstants.State ledState){
        state = ledState;
    }
}