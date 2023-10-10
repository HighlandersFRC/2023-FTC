package org.firstinspires.ftc.teamcode.grab;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Drive Gripper", group = "Exercises")
//@Disabled
    public class DriveWithGripper extends LinearOpMode {

        CRServo contServo;
    Servo armServo;
        double contPower;

        @Override
        public void runOpMode() throws InterruptedException {
            contServo = hardwareMap.crservo.get("contServo");
            armServo = hardwareMap.servo.get("ArmServo");
            telemetry.addData("Mode", "waiting");
            telemetry.update();

            // wait for start button.

            waitForStart();

            while (opModeIsActive()) {

                telemetry.addData("Mode", "running");

                if (gamepad1.x) {
                    contPower =1;
                }
                else if (gamepad1.b)
                    contPower = -1;

                if (gamepad2.a){
                armServo.setPosition(180);
                    ;
                }
                else
                    contPower = 0.0;

                contServo.setPower(contPower);

                telemetry.update();
                idle();
            }
        }
    }
