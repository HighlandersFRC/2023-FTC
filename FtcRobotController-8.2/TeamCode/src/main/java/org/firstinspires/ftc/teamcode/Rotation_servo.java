package org.firstinspires.ftc.teamcode;

import android.os.PowerManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test_Servo", group="Linear OpMode")
//@Disabled
public class Rotation_servo extends LinearOpMode {

        private ElapsedTime runtime = new ElapsedTime();
        Servo servo;
        double servoPosition = 0.0;

        @Override

        public void runOpMode() throws InterruptedException {
                CRServo contServo = hardwareMap.crservo.get("armServo");

                contServo.resetDeviceConfigurationForOpMode();

                waitForStart();


                while (opModeIsActive()) {


                        double cntPower;
                        if (gamepad1.dpad_up) {
                                cntPower = -0.45;
                                telemetry.addData("Keypad", "dpad_up clicked. power = " + cntPower);
                        } else if (gamepad1.dpad_down) {
                                cntPower = 0.45;
                                telemetry.addData("Keypad", "dpad_down clicked. power = " + cntPower);
                        } else {
                                cntPower = 0.0;
                                telemetry.addData("Keypad", "Nothing pressed. power = " + cntPower);

                        }

                        contServo.setPower(cntPower);

                        telemetry.update();


                }
        }
}