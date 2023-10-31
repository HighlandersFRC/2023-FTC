package org.firstinspires.ftc.teamcode.grab;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp

/* At this point, Joystick X/Y (strafe/forwrd) vectors have been */
/* rotated by the gyro angle, and can be sent to drive system */
public class NavxMXP extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        NavxMicroNavigationSensor navxMXP;
        navxMXP = (NavxMicroNavigationSensor) hardwareMap.i2cDevice.get(NavxMXP.class+"NavxMXP");

        waitForStart();
        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;
            telemetry.addData("test angle",navxMXP.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));


        }
    }
}