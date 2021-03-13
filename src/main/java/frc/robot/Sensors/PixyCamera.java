package frc.robot.Sensors;


import java.util.ArrayList;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;



//USING PIXY CAMERA
//Partially recycled
//There are two different cameras! The one that determines color 

public class PixyCamera {

    private static AnalogInput pixy;
    public static AnalogOutput led;
    public static boolean isLedOn = false;
    /*
    private static Pixy2CCC ccc;
    private static ArrayList<Block> blockBuffer;
    */

    public static void initialize() {

       // pixy = new AnalogInput(Constants.PIXY_CAM);
        led = new AnalogOutput(0);
       // switchLED(false);

        // ccc = new Pixy2().getCCC();

    }
    

    public static double getTargetLocation() {
        // return (double)(getHighPort().getX()) / 315 * 2 - 1;
        return (pixy.getAverageVoltage() / 3.3 * 2) - 1;
    }

    
}
