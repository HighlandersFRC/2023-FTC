package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp

public class Test extends LinearOpMode {
    private DcMotor motorTest;

    private TouchSensor touchSensor;

    PID1 PIDTest = new PID1();




    @Override

    public void runOpMode() {

        PIDTest.setSetPoint(168);



        motorTest = hardwareMap.get(DcMotor.class, "Left_Back");

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

// Wait for the game to start (driver presses PLAY)

        waitForStart();{
            motorTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

// run until the end of the match (driver presses STOP)
//28 ticks per full revolve
        while (opModeIsActive()) {
            PIDTest.setPID(0.75,0.2 ,0.3);
            motorTest.setPower(-1);
            motorTest.setTargetPosition(PIDTest.getSetPoint());
            motorTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           /* if (motorTest.getCurrentPosition() <= PIDTest.getSetPoint() - 3 || motorTest.getCurrentPosition() <= + 3){
                //motorTest.setPower(0);
            }*/
            telemetry.addData("Current Position", motorTest.getCurrentPosition());
            telemetry.update();
        }
    }
}