package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.CommandGroup;
import org.firstinspires.ftc.teamcode.Commands.DriveForward;
import org.firstinspires.ftc.teamcode.Commands.Scheduler;
import org.firstinspires.ftc.teamcode.Commands.Wait;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous

public class Autonomous extends LinearOpMode {
    private DcMotor Left_Front;
    private DcMotor Right_Front;
    private DcMotor Left_Back;
    private DcMotor Right_Back;

    Scheduler scheduler = new Scheduler();

    @Override
    public void runOpMode() {
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");

        waitForStart();
        //scheduler.add(new DriveForward(hardwareMap, 1, 1000));
        scheduler.add(new CommandGroup(
                scheduler,
                new DriveForward(hardwareMap, 1, 1000),
                new Wait(2000),
                new DriveForward(hardwareMap, -1, 1000)
        ));
        while (opModeIsActive()) {
            scheduler.update();
        }
    }
}
