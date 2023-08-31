package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp

public class test extends LinearOpMode {
    //Initialise motors, classes, and sensors here
    private DcMotor Arm;
    private DcMotor Left_Front;
    private DcMotor Right_Front;
    private DcMotor Left_Back;
    private DcMotor Right_Back;

    @Override

    public void runOpMode() {
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");
        Arm = hardwareMap.dcMotor.get("Arm");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double vertical;
            double horizontal;
            double pivot;
            double arm;
            vertical = -gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            pivot = -gamepad1.right_stick_x;
            vertical = -gamepad1.left_stick_y;

            //Mecanum drive
            Right_Front.setPower(pivot + (vertical + horizontal));
            Right_Back.setPower(pivot + (vertical - horizontal));
            Left_Back.setPower(pivot + (-vertical - horizontal));
            Left_Front.setPower(pivot + (-vertical + horizontal));

            //Code for arm

            if (gamepad1.y) {
                Arm.setTargetPosition(-900);
                Arm.setPower(-1);

            }
        }
            if (gamepad1.a) {
                Arm.setTargetPosition(900);
                Arm.setPower(1);


            if (gamepad1.b) {
                Arm.setTargetPosition(-450);
                Arm.setPower(-.5);

            }
        }
    }
}

