package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PID;

public class wristDown extends Command{
    public Servo WristServo;
    public double target;

    public wristDown(HardwareMap hardwareMap, double targetPos){
    WristServo = hardwareMap.servo.get("WristServo");
    targetPos = this.target;

    }
    public void start() {
        WristServo.setDirection(Servo.Direction.FORWARD);
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
