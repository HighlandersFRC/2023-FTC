package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class wristDisable extends Command{
    public Servo WristServo;

    public wristDisable(HardwareMap hardwareMap){

        WristServo = hardwareMap.servo.get("WristServo");
    }
    public void start() {
        WristServo.setDirection(Servo.Direction.FORWARD);
        WristServo.setPosition(1);

    }

    public void execute() {
    }

    public void end() {

    }

    public boolean isFinished() {
        return true;
    }
}
