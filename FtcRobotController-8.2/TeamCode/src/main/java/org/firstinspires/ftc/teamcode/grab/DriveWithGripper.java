package org.firstinspires.ftc.teamcode.grab;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PID1;


@TeleOp(name = "Drive Gripper", group = "Exercises")
//@Disabled
    public class DriveWithGripper extends LinearOpMode {


        CRServo contServo;
        float leftY, rightY;
        double armPosition, gripPosition, contPower;
        double MIN_POSITION = 0, MAX_POSITION = 1;
        PID1 PID = new PID1();
        // called when init button is  pressed.
        @Override
        public void runOpMode() throws InterruptedException {



            contServo = hardwareMap.crservo.get("contServo");

            telemetry.addData("Mode", "waiting");
            telemetry.update();

            // wait for start button.

            waitForStart();

            armPosition = .5;                   // set arm to half way up.
            gripPosition = MAX_POSITION;        // set grip to full open.

            while (opModeIsActive()) {


               ;
                telemetry.addData("Mode", "running");

                PID.setPID(.01,.0 ,.0);
                if (gamepad1.x) {

                    contPower =1;
                }
                else if (gamepad1.b)
                    contPower = -1;
                else
                    contPower = 0.0;


                contServo.setPower(contPower);

                telemetry.update();
                idle();
            }
        }
    }
