package org.firstinspires.ftc.teamcode;

import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    private DcMotor Left_Front = null;
    private DcMotor Right_Front = null;
    private DcMotor Left_Back = null;
    private DcMotor Right_Back = null;

    @Override
    public void runOpMode() {
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");

        waitForStart();

        if (opModeIsActive()) {

            Left_Front.setPower((double) 0.5);
            Right_Front.setPower((double) 0.5);
            Left_Back.setPower((double) 0.5);
            Right_Back.setPower((double) 0.5);

        }
    }
}
