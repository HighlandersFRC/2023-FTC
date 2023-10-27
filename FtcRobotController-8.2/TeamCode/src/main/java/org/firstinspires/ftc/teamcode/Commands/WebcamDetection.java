
package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class WebcamDetection extends Command{
    public WebcamName Webcam1;
    Boolean Done = false;
    public WebcamDetection(HardwareMap hardwareMap){
        Webcam1 = hardwareMap.get(WebcamName.class,"webcam1");
        
    }

    public void start(){
    }

    public void execute(){
        Done = true;
    }

    public void end(){

    }

    public boolean isFinished(){
        if (Done){
            return true;
        }
        return false;
    }
}
