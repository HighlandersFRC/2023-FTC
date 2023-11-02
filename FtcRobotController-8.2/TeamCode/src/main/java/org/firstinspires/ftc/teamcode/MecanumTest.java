package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MecanumTest extends LinearOpMode {
    public DcMotor Left_Back;
    public DcMotor Right_Back;
    public DcMotor Left_Front;
    public DcMotor Right_Front;
    @Override
    public void runOpMode() {

        waitForStart();
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");

        while (opModeIsActive()) {

            if (Math.abs(leftStickX) < 0.0001){
                Left_Front.setPower(0);
                Left_Back.setPower(0);
                Right_Front.setPower(0);
                Right_Back.setPower(0);
            }
            if (Math.abs(leftStickY) < 0.0001){
                Left_Front.setPower(0);
                Left_Back.setPower(0);
                Right_Front.setPower(0);
                Right_Back.setPower(0);
            }

            double y = leftStickY;
            double x = leftStickX;
            double rx = rightStickX;


            double frontLeftPower = (y + x + rx);
            double backLeftPower = (y - x + rx);
            double frontRightPower = (y - x - rx);
            double backRightPower = (y + x - rx);

            Left_Front.setPower(frontLeftPower);
            Left_Back.setPower(backLeftPower);
            Right_Front.setPower(frontRightPower);
            Right_Back.setPower(backRightPower);



        }




    }
}
