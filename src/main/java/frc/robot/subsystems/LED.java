package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LED implements Subsystem{
    
    CANifier canifier = new CANifier(0);
    public static LED instance;
    private LED(){

    }   
    public static LED getInstance(){
        if(instance == null){instance = new LED();}
        return instance;
    }
    public void setRGB(double red, double green, double blue){
       canifier.setLEDOutput(red, LEDChannel.LEDChannelA);
       canifier.setLEDOutput(green, LEDChannel.LEDChannelB);
       canifier.setLEDOutput(blue, LEDChannel.LEDChannelC);
    }

    public void setColor(Color color){
        setRGB(color.red, color.green, color.blue);
    }

}