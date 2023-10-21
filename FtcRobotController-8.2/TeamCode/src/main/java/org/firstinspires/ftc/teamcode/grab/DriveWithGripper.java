package org.firstinspires.ftc.teamcode.grab;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Drive Gripper", group = "Exercises")
//@Disabled
    public class DriveWithGripper extends LinearOpMode {

        Servo armServo;
        Servo wristServo;

        double contPower;


        // called when init button is  pressed.

        @Override

        public void runOpMode() throws InterruptedException {
            wristServo = hardwareMap.servo.get("wristServo");
            armServo = hardwareMap.servo.get("armServo");
            telemetry.addData("Mode", "waiting");

            telemetry.update();


            // wait for start button.

            waitForStart();

            while (opModeIsActive()) {
                wristServo.scaleRange(0,0.1);
                telemetry.addData("time",getRuntime());

                telemetry.addData("ArmPosition",wristServo.getPosition());
                telemetry.addData("Mode", "running");

                if (gamepad1.x) {

                    wristServo.setPosition(1);
                }
                if(gamepad1.y) {

                    wristServo.setPosition(.001);
                }
                if (gamepad1.a){
                    armServo.setPosition(0);
                }
                    if (gamepad1.b) {
                        armServo.setDirection(Servo.Direction.FORWARD);
                        armServo.setPosition(110);

                    }
                else {

                }
                telemetry.update();
                idle();
            }
        }
    }
