package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ArmConstants;
import org.firstinspires.ftc.teamcode.PID;

public class PixelTray extends Command {
    public CRServo armServo;
    public CRServo armServo2;
    public long time;
    public double speed;
    public long endTime;
    public String LR;
    public DcMotor Arm_Motor;
    PID ArmPID = new PID(0.001, 0, 0);



    public PixelTray(HardwareMap hardwareMap, long Time, double Speed, String LR) {
        armServo = hardwareMap.crservo.get("intakeServo");
        armServo2 = hardwareMap.crservo.get("intakeServo2");
        this.speed = Speed;
        this.time = Time;
        this.LR = LR;
        Arm_Motor = hardwareMap.dcMotor.get("Arm_Motor");
        ArmPID.setSetPoint(ArmConstants.prevSetPoint);
    }

    public void start() {
        if (LR == "L") {
            armServo.setPower(speed);
        }else
            if(LR == "R"){
                armServo2.setPower(-speed);
        }else
            if (LR == "LR"){
                armServo.setPower(speed);
                armServo2.setPower(-speed);
            }


        endTime = System.currentTimeMillis() + time;
    }
    public void execute() {
/*        ArmPID.updatePID(Arm_Motor.getCurrentPosition());
        Arm_Motor.setPower(ArmPID.getResult());*/
    }

    public void end() {
        armServo.setPower(0);
        armServo2.setPower(0);
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