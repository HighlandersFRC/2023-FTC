package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration, ok?
        DcMotor Left_Front = hardwareMap.dcMotor.get("Left_Front");
        DcMotor Left_Back = hardwareMap.dcMotor.get("Left_Back");
        DcMotor Right_Front = hardwareMap.dcMotor.get("Right_Front");
        DcMotor Right_Back = hardwareMap.dcMotor.get("Right_Back");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        Right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Back.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y * 0.95; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 0.95;
            double rx = -gamepad1.right_stick_x * 0.95;


            // This button choice was made so that i   t is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }


            double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double botHeadingRadian = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (botHeadingRadian != 0) {
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeadingRadian) - y * Math.sin(botHeadingRadian);// Changed to positive due to things(change back when need)
                double rotY = x * Math.sin(-botHeadingRadian) + y * Math.cos(botHeadingRadian);//Changed to positive due to things(change back when need)

                rotX = rotX * 1.1;  // Counteract imperfect strafing
                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

            Left_Front.setPower(frontLeftPower);
            Left_Back.setPower(backLeftPower);
            Right_Front.setPower(frontRightPower);
            Right_Back.setPower(backRightPower);
            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.addData("rx", rx);
            telemetry.addData("rotY", rotY);
            telemetry.addData("rotX", rotX);
            telemetry.addData("parameters", parameters);
            telemetry.addData("IMU", imu);
            telemetry.addData("denominator", denominator);
            telemetry.addData("botHeading", botHeading);
            telemetry.addData("botHeadingRadian", botHeadingRadian);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.addData("time", time);
            telemetry.update();



        }
        else continue;
        }
        // Find a motor in the hardware map named "Arm Motor"
        double CPR = 288;

        int positionLeft_Front = Left_Front.getCurrentPosition();
        positionLeft_Front = Left_Front.getCurrentPosition();
        double revolutionsLeft_Front = positionLeft_Front/CPR;

        int positionLeft_Back = Left_Back.getCurrentPosition();
        positionLeft_Back = Left_Back.getCurrentPosition();
        double revolutionsLeft_Back = positionLeft_Back/CPR;

        int positionRight_Front = Right_Front.getCurrentPosition();
        positionRight_Front = Right_Front.getCurrentPosition();
        double revolutions = positionRight_Front/CPR;

        int positionRight_Back = Right_Back.getCurrentPosition();
        positionLeft_Back = Right_Back.getCurrentPosition();
        double revolutionsRight_Back = positionRight_Back/CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        // Reset the motor encoder so that it reads zero ticks


        waitForStart();

        while (opModeIsActive()) {

            // Get the current position of the armMotor
            positionLeft_Front = Left_Front.getCurrentPosition();
            positionLeft_Back = Left_Back.getCurrentPosition();
            positionRight_Front = Right_Front.getCurrentPosition();
            positionRight_Back = Right_Back.getCurrentPosition();
            // Get the target position of the armMotor
            double desiredPositionLeft_Front = Left_Front.getTargetPosition();
            double desiredPositionLeft_Back = Left_Back.getTargetPosition();
            double desiredPositionRight_Front = Right_Front.getTargetPosition();
            double desiredPositionRight_Back = Right_Back.getTargetPosition();


            // Show the position of the armMotor on telemetry
            telemetry.addData("Encoder Position Left_Front", Left_Front);
            telemetry.addData("Encoder Position Left_Back", Left_Back);
            telemetry.addData("Encoder Position Right_Front", Right_Front);
            telemetry.addData("Encoder Position Right_Back", Right_Back);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Desired Position Left_Front", desiredPositionLeft_Front);
            telemetry.addData("Desired Position Left_Back", desiredPositionLeft_Back);
            telemetry.addData("Desired Position Right_Front", desiredPositionRight_Front);
            telemetry.addData("Desired Position Right_Back", desiredPositionRight_Back);

            telemetry.addData("position Left_Front", positionLeft_Front);
            telemetry.addData("position Left_Back", positionLeft_Back);
            telemetry.addData("position Right_Front", positionRight_Front);
            telemetry.addData("position Right_Back", positionRight_Back);

            telemetry.addData("angle", angle);
            telemetry.addData("angleNormalized", angleNormalized);
            telemetry.update();
        }
    }
}