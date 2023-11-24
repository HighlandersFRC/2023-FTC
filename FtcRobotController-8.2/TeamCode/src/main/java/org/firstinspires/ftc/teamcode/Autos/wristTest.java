package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandGroup;
import org.firstinspires.ftc.teamcode.Commands.MoveWrist;
import org.firstinspires.ftc.teamcode.Commands.Scheduler;

@Autonomous
public class wristTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler scheduler = new Scheduler();
        waitForStart();
        scheduler.add(new CommandGroup(scheduler, new MoveWrist(hardwareMap, 1)));
        while (opModeIsActive()) {
scheduler.update();
        }
    }
}
