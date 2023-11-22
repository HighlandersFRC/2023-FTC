package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeServoUp extends Command{
    public Servo LServo;
    public Servo RServo;
    Boolean Done = false;
    public IntakeServoUp(HardwareMap hardwareMap){
        LServo = hardwareMap.servo.get("LServo");
        RServo = hardwareMap.servo.get("RServo");
    }

    public void start(){
        LServo.setPosition(0.7);
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
