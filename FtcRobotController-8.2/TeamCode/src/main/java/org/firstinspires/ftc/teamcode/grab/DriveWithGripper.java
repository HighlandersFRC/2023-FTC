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

                // check the gamepad buttons and if pressed, increment the appropriate position
                // variable to change the servo location.

                // move arm down on A button if not already at lowest position.
                if (gamepad1.a && armPosition > MIN_POSITION) armPosition -= .01;

                // move arm up on B button if not already at the highest position.
                if (gamepad1.b && armPosition < MAX_POSITION) armPosition += .01;

                // open the gripper on X button if not already at most open position.
                if (gamepad1.x && gripPosition < MAX_POSITION) gripPosition = gripPosition + .01;

                // close the gripper on Y button if not already at the closed position.
                if (gamepad1.y && gripPosition > MIN_POSITION) gripPosition = gripPosition - .01;

                // Set continuous servo power level and direction.
                PID.setPID(.01,.0 ,.0);
                if (gamepad1.x) {

                    contPower =1;
                }
                else if (gamepad1.b)
                    contPower = -1;
                else
                    contPower = 0.0;

                // set the servo position/power values as we have computed them.

                contServo.setPower(contPower);


                //telemetry.addData("arm servo", String.format("position=%.2f  actual=%.2f", armPosition,
                //    armServo.getPosition()));

                //telemetry.addData("grip servo", String.format("position=%.2f  actual=%.2f", gripPosition,
                //    gripServo.getPosition()));

                //telemetry.addData("arm servo", "position=%.2f  actual=%.2f", armPosition, armServo.getPosition());

                //telemetry.addData("grip servo", "position=%.2f  actual=%.2f", gripPosition, gripServo.getPosition());

                telemetry.update();
                idle();
            }
        }
    }
