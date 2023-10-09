package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.PID1;

@TeleOp
//23477
public class Mecanum extends LinearOpMode {
    private DcMotor Left_Front;
    private DcMotor Right_Front;
    private DcMotor Left_Back;
    private DcMotor Right_Back;
    private DcMotor Left_Intake;
    private DcMotor Right_Intake;
    private DcMotor Arm1;
    private DcMotor Arm2;

    // private OpticalDistanceSensor distance_Sensor;
    PID1 PID = new PID1(0.1, 0, 0);

    @Override
    public void runOpMode() {

        //distance_Sensor = hardwareMap.opticalDistanceSensor.get("distanceSensor");
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");
        Left_Intake = hardwareMap.dcMotor.get("Left_Intake");
        Right_Intake = hardwareMap.dcMotor.get("Right_Intake");
        Arm1 = hardwareMap.dcMotor.get("Arm1");
        Arm2 = hardwareMap.dcMotor.get("Arm2");

// Wait for the game to start (driver presses PLAY)

        waitForStart();

        if (isStopRequested()) return;

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;

            PID.setMaxOutput(1);
            PID.setMinOutput(-1);
            PID.setPID(0.003,0 ,0.001);
            PID.updatePID(Arm1.getCurrentPosition());
            Arm1.setPower(PID.getResult() - 0.001);
            Arm2.setPower(PID.getResult() - 0.001);
            if (gamepad1.a){
                PID.setSetPoint(-350);
            }
            if (gamepad1.b){
                PID.setSetPoint(250);
            }

            if (gamepad1.x){
                PID.setSetPoint(-75);
            }
            if (gamepad1.y){
                PID.setSetPoint(-175);
            }
            Right_Intake.setPower(-leftTrigger);
            Left_Intake.setPower(leftTrigger);
            Right_Intake.setPower(-rightTrigger);
            Left_Intake.setPower(rightTrigger);

            double y = -gamepad1.left_stick_y / 2 * 0.75;
            double x = gamepad1.left_stick_x * 1.1 * 0.75;
            double rx = -gamepad1.right_stick_x;

            if (Math.abs(gamepad1.left_stick_x) < 0.00001){
                Left_Front.setPower(0);
                Left_Back.setPower(0);
                Right_Front.setPower(0);
                Right_Back.setPower(0);
            }
            if (Math.abs(gamepad1.left_stick_y) < 0.00001){
                Left_Front.setPower(0);
                Left_Back.setPower(0);
                Right_Front.setPower(0);
                Right_Back.setPower(0);
            }
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            Left_Front.setPower(frontLeftPower);
            Left_Back.setPower(backLeftPower);
            Right_Front.setPower(-frontRightPower);
            Right_Back.setPower(-backRightPower);

            telemetry.addLine("Motor Positions");
            telemetry.addData("Front-Left Position", Left_Front.getCurrentPosition());
            telemetry.addData("Front-Right Position", Right_Front.getCurrentPosition());
            telemetry.addData("Back-Left Position", Left_Back.getCurrentPosition());
            telemetry.addData("Back-Right Position", Right_Back.getCurrentPosition());
            telemetry.addData("Arm One Encoder", Arm1.getCurrentPosition());
            telemetry.addData("Arm Two Encoder", Arm2.getCurrentPosition());

            telemetry.addLine("");
            telemetry.addLine("Controller Inputs");
            telemetry.addData("Left Trigger", leftTrigger);
            telemetry.addData("Real Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", rightTrigger);
            telemetry.addData("Real Right Trigger", gamepad1.right_trigger);
            telemetry.addData("PID Result", PID.getResult());

            telemetry.addLine("");
            //telemetry.addData("Distance Sensor", distance_Sensor);
            telemetry.update();
        }
    }
}