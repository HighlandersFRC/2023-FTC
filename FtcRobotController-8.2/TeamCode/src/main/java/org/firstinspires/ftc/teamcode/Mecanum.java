package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp

public class Mecanum extends LinearOpMode {
    private DcMotor Left_Front = null;
    private DcMotor Right_Front = null;
    private DcMotor Left_Back = null;
    private DcMotor Right_Back = null;

    PID1 PID = new PID1();

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
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.09999;
            double rx = gamepad1.right_stick_x;

/*           if (gamepad1.left_stick_y < 0.001 | gamepad1.left_stick_y > -0.001){
                y = 0;
            }
            if (gamepad1.left_stick_x < 0.001 | gamepad1.left_stick_x > -0.001){
                x = 0;
            }
*/
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

            telemetry.addLine("");
            telemetry.addLine("Controller Inputs");
            telemetry.addData("Left Stick X", x);
            telemetry.addData("Left Stick Y", y);

            telemetry.update();

            if (gamepad1.a){
                Left_Front.setPower(1);
            }
            if (gamepad1.b){
                Left_Back.setPower(1);
            }
            if (gamepad1.y){
                Right_Front.setPower(1);
            }
            if (gamepad1.x){
                Right_Back.setPower(1);
            }
        }
    }
}