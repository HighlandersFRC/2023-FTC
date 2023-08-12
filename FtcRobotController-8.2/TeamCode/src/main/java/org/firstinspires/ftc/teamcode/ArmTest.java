package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp

public class ArmTest extends LinearOpMode {


    private DcMotor Arm = null;
    @Override

    public void runOpMode() {

        Arm = hardwareMap.get(DcMotor.class, "Left_Back");


// Wait for the game to start (driver presses PLAY)

        waitForStart();{

        }

// run until the end of the match (driver presses STOP)
//28 ticks per full revolve
        while (opModeIsActive()) {
            double Yvariable = gamepad1.left_stick_y;
            double ArmPower = Yvariable / 2.5;
            Arm.setPower(ArmPower);
            telemetry.addData("Current Position", Arm.getCurrentPosition());
            telemetry.addData("Get Current Power", Arm.getPower());
            telemetry.addData("Y", Yvariable);
            telemetry.addData("ControllerInput", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}