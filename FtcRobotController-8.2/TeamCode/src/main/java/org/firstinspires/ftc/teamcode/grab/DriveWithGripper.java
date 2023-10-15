package org.firstinspires.ftc.teamcode.grab;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Drive Gripper", group = "Exercises")
//@Disabled
    public class DriveWithGripper extends LinearOpMode {

        Servo armServo;
        CRServo contServo;

        double contPower;

        //PID1 PID = new PID1(1,1,0);
        // called when init button is  pressed.
        @Override
        public void runOpMode() throws InterruptedException {
            contServo = hardwareMap.crservo.get("contServo");
            armServo = hardwareMap.servo.get("armServo");
            telemetry.addData("Mode", "waiting");
            telemetry.update();

            // wait for start button.

            waitForStart();

            while (opModeIsActive()) {


                telemetry.addData("Mode", "running");

                if (gamepad1.x) {

                    contPower =1;
                    contServo.setPower(contPower);
                }
                if(gamepad1.y) {
                    contPower=-1;

                }
                if (gamepad1.a){
                    armServo.setPosition(0);
                }
                    if (gamepad1.b) {
                    armServo.setPosition(220);
                }
                else
                    contPower = 0.0;





                telemetry.update();
                idle();
            }
        }
    }
