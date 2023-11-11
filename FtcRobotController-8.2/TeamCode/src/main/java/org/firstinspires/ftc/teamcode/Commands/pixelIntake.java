package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class pixelIntake extends Command {
    public CRServo armServo;
    public CRServo armServo2;
    public long time;
    public double speed;
    public long endTime;


    public pixelIntake(HardwareMap hardwareMap, long Time, double Speed) {
        armServo = hardwareMap.crservo.get("armServo");
        armServo2 = hardwareMap.crservo.get("armServo2");
        this.speed = Speed;
        this.time = Time;
    }

    public void start() {
        armServo.setPower(speed);
        armServo2.setPower(-speed);
        endTime = System.currentTimeMillis() + time;
    }
    public void execute() {

    }

    public void end() {
        armServo.setPower(speed);
        armServo2.setPower(-speed);
    }

    public boolean isFinished() {
        if (System.currentTimeMillis() >= endTime){
            return true;
        }
        else {
            return false;
        }
    }
}