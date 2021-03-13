package frc.robot.Sensors;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoException;

public class Camera {

    static UsbCamera cam0;
    
    static public void initialize() {
        cam0 = CameraServer.getInstance().startAutomaticCapture(0);

        try {
        cam0.setResolution(640,480);
        //cam0.setResolution(1280, 720);
        //cam0.setFPS(24);
        } catch(VideoException e) {
            //Shhhhh
        }
    }

}