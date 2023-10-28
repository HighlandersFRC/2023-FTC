package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PID1 PID = new PID1();
        // Declare our motors
        DcMotor Left_Front = hardwareMap.dcMotor.get("Left_Front");
        DcMotor Left_Back = hardwareMap.dcMotor.get("Left_Back");
        DcMotor Right_Front = hardwareMap.dcMotor.get("Right_Front");
        DcMotor Right_Back = hardwareMap.dcMotor.get("Right_Back");
        //I2cDevice navx = hardwareMap.i2cDevice.get("NavX");
        DcMotor Left_Intake = hardwareMap.dcMotor.get("Left_Intake");
        DcMotor Right_Intake = hardwareMap.dcMotor.get("Right_Intake");
        Servo LServo = hardwareMap.servo.get("LServo");
        Servo RServo = hardwareMap.servo.get("RServo");
        DcMotor Arm1 = hardwareMap.dcMotor.get("Arm1");
        DcMotor Arm2 = hardwareMap.dcMotor.get("Arm2");
        CRServo intakeServo = hardwareMap.crservo.get("intakeServo");


        Right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Back.setDirection(DcMotorSimple.Direction.REVERSE);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();
        
        long timeElapsed = 0;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            timeElapsed  = System.currentTimeMillis();
            if (timeElapsed >= 1){

            }
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;
            LServo.scaleRange(-180, 180);
            RServo.scaleRange(-180, 180);
            double intakePower = (rightTrigger - leftTrigger) * 0.75;

            PID.setMaxOutput(1);
            PID.setMinOutput(-1);
            PID.setPID(0.003,0 ,0.001);
            PID.updatePID(Arm1.getCurrentPosition());
            Arm1.setPower(PID.getResult() - 0.001);
            Arm2.setPower(PID.getResult() - 0.001);

            if (!(rightTrigger == 0)){
                intakeServo.setPower(1);
            }
            if (!(leftTrigger == 0)){
                intakeServo.setPower(-1);
            }
            /*
            if (gamepad1.dpad_left){
                intakeServo.setPower(-1);
            }
            else {
                if (gamepad1.dpad_right){
                    intakeServo.setPower(1);
                }
                else
                {
                    intakeServo.setPower(0);
                }
            }
      */
            if (gamepad1.dpad_up){
                LServo.setPosition(-120);
                RServo.setPosition(120);
            }
            if (gamepad1.dpad_down){
                LServo.setPosition(1);
                RServo.setPosition(-1);
            }
            if (gamepad2.a){
                PID.setSetPoint(-350);

            }
            if (gamepad2.b){
                PID.setSetPoint(250);
            }

            if (gamepad2.x){
                PID.setSetPoint(-75);
            }
            if (gamepad2.y){
                PID.setSetPoint(-175);
            }
            Right_Intake.setPower(-intakePower);

            if (gamepad1.options) {
                imu.resetYaw();
            }


            double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double botHeadingRadian = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (botHeadingRadian != 0) {
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(botHeadingRadian) - y * Math.sin(botHeadingRadian);// Changed to positive due to things(change back when need)
                double rotY = x * Math.sin(botHeadingRadian) + y * Math.cos(botHeadingRadian);//Changed to positive due to things(change back when need)

                rotX = rotX * 1.1;  // Counteract imperfect strafing
                // Denominator is the largest motor power (absolute value) or 1rmn
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                Left_Front.setPower(-frontLeftPower);
                Left_Back.setPower(-backLeftPower);
                Right_Front.setPower(-frontRightPower);
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
                telemetry.update();}
        }
    }
}

