package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MoveWrist extends Command{
    public Servo WristServo;
    public double target;

    public MoveWrist(HardwareMap hardwareMap, double targetPos){
    WristServo = hardwareMap.servo.get("WristServo");
    targetPos = this.target;

    }
    public void start() {
        WristServo.setDirection(Servo.Direction.REVERSE);
        WristServo.setPosition(target);
    }

    public void execute() {
    }

    public void end() {

    }

    public boolean isFinished() {
        return true;
    }
}
