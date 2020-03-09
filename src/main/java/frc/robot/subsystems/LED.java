package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LEDConstants;
 
public class LED implements Subsystem{
    
    CANifier canifier = new CANifier(0);
    private static LED instance;
    public static LEDConstants.State state;
    public static Color color;

    private LED(){
        register();
    }  

    @Override 
    public void periodic(){
        switch(state){
            case SHOOTING: { setColor(Color.kBlack, 0.0); break; }
            case INTAKE: { setColor(Color.kBlue, 0.0); break; }
            case ENABLED: { setColor(Color.kDarkTurquoise, 0.0); break; } 
            case DISABLED: { setColor(Color.kGreen, 0.0); break; }
            case ERROR: { setColor(Color.kRed, 0.0); break; }
            case NULL: { setRGB(0,0,0,0.0);}
        }
    }

    public static LED getInstance(){
        if(instance == null){instance = new LED();}
        return instance;
    }
    public void setRGB(double red, double green, double blue, double blinkTiming){
       canifier.setLEDOutput(red, LEDChannel.LEDChannelA);
       canifier.setLEDOutput(green, LEDChannel.LEDChannelB);
       canifier.setLEDOutput(blue, LEDChannel.LEDChannelC);
    }

    public void setColor(Color color, double blinkTiming){
        setRGB(color.red, color.green, color.blue, blinkTiming);
    }
    public void setState(LEDConstants.State ledState){
        state = ledState;
    }
}