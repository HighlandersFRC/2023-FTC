package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.CommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Commands.Scheduler;
import org.firstinspires.ftc.teamcode.Commands.Wait;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous

public class Autonomous extends LinearOpMode {

    Scheduler scheduler = new Scheduler();
    Scheduler scheduler2 = new Scheduler();
    @Override
    public void runOpMode() {
        waitForStart();
        //scheduler.add(new DriveForward(hardwareMap, 1, 1000));
        scheduler.add(new CommandGroup(
                scheduler,
                new Drive(hardwareMap, 1, 1000),
                new Wait(2000),
                new Drive(hardwareMap, -1, 1000)
        ));
        scheduler2.add(new CommandGroup(
                scheduler2,
                new Wait(1000)
        ));
        while (opModeIsActive()) {
            scheduler.update();
        }
    }
}
