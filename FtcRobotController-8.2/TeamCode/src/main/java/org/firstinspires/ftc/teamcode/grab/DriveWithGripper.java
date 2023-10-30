package org.firstinspires.ftc.teamcode.grab;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp

//@Disabled
    public class DriveWithGripper extends LinearOpMode {
    //make servo
    Servo wristServo, armServo;
    Servo gripServo;

    // called when init button is  pressed.

    @Override

    public void runOpMode() throws InterruptedException {
        gripServo = hardwareMap.servo.get("gripServo");
        wristServo = hardwareMap.servo.get("wristServo");
        armServo = hardwareMap.servo.get("armServo");
        NavxMicroNavigationSensor microNavigationSensor = (NavxMicroNavigationSensor) hardwareMap.i2cDevice.get("NavX");
        telemetry.update();
        // wait for start button.

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("time", getRuntime());
            telemetry.addData("ArmPosition", gripServo.getPosition());
            telemetry.addData("Mode", "running");

            gripServo.setPosition(gamepad1.left_stick_y);

            if (gamepad1.a) {
                wristServo.setPosition(60);

            }
            if (gamepad1.b) {
                wristServo.setPosition(180);
            }
            if (gamepad1.left_bumper) {
                armServo.setPosition(60);
            }
            if (gamepad1.right_bumper) {
                armServo.setPosition(180);
            } else {
                double v =gripServo.getPosition() ;
                gripServo.setPosition(v);
            }
            telemetry.update();
            idle();

        }

    }

}