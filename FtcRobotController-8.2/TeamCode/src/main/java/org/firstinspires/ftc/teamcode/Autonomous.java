package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.Arm;
import org.firstinspires.ftc.teamcode.Commands.CommandGroup;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Commands.Rotate;
import org.firstinspires.ftc.teamcode.Commands.Scheduler;
import org.firstinspires.ftc.teamcode.Commands.Turn;
import org.firstinspires.ftc.teamcode.Commands.Wait;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous

public class Autonomous extends LinearOpMode {

    Scheduler scheduler = new Scheduler();
    DcMotor Arm1;

    @Override
    public void runOpMode() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        Arm1 = hardwareMap.dcMotor.get("Arm1");
        waitForStart();
        scheduler.add(new CommandGroup(scheduler,
                new Turn(hardwareMap, -90),
                new Wait(3000)
               //new Drive(hardwareMap, 1, 1000)
                //new Rotate(hardwareMap, -90),
                //new Drive(hardwareMap, -1, 1000)
        ));
        while (opModeIsActive())
        {
            telemetry.addData("ArmPosition", Arm1.getCurrentPosition());
            telemetry.addData("IMU", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            scheduler.update();
        }
    }
}