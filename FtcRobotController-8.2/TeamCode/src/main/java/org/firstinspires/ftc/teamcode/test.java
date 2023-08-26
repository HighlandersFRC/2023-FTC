package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp

public class test extends LinearOpMode {
    //Initialise motors, classes, and sensors here

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
// Wait for the game to start (driver presses PLAY)
        waitForStart();

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double LeftY = gamepad1.left_stick_y;
            double LeftX = gamepad1.left_stick_x;
            Left_Front.setPower(LeftY);
            Right_Front.setPower(-LeftY);
            Left_Back.setPower(LeftY);
            Right_Back.setPower(-LeftY);
            Left_Front.setPower(LeftX);
            Right_Back.setPower(LeftX);
        }
    }
}