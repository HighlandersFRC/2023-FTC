package org.firstinspires.ftc.teamcode.Commands;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turn extends Command {
    private DcMotor Left_Back;
    private DcMotor Right_Back;
    private DcMotor Left_Front;
    private DcMotor Right_Front;

    String Direction;

    public Turn(HardwareMap hardwareMap, String Direction){
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");
        this.Direction = Direction;

    }
    public void start(){
        Left_Front.setPower(speed);
        Right_Front.setPower(-speed);
        Left_Back.setPower(speed);
        Right_Back.setPower(-speed);
        endTime = System.currentTimeMillis() + time;
    }
    public void execute(){

    }
    public void end(){
        Left_Front.setPower(0);
        Right_Front.setPower(0);
        Left_Back.setPower(0);
        Right_Back.setPower(0);
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
