package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public abstract class WhitePixelDetection extends Command{
    public Servo LServo;
    public Servo RServo;
    Boolean Done = false;
    public WhitePixelDetection(HardwareMap hardwareMap){
        WebcamName WebcamName = (hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "webcam1"));
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
