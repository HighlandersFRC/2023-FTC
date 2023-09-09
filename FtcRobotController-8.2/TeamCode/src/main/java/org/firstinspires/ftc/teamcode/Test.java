package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class Test extends LinearOpMode {
    //Initialise motors, classes, and sensors here
    private DcMotor Arm1;
   // private DcMotor Left_Front;
   // private DcMotor Right_Front;
   //private DcMotor Left_Back;
   //private DcMotor Right_Back;

    PID1 PID = new PID1();

    @Override

    public void runOpMode() {
        //Left_Front = hardwareMap.dcMotor.get("Left_Front");
        //Right_Front = hardwareMap.dcMotor.get("Right_Front");
        //Left_Back = hardwareMap.dcMotor.get("Left_Back");
        //Right_Back = hardwareMap.dcMotor.get("Right_Back");
        Arm1 = hardwareMap.dcMotor.get("Arm1");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double vertical;
            double horizontal;
            double pivot;
            double arm1;
            //vertical = -gamepad1.left_stick_y;
            //horizontal = gamepad1.left_stick_x;
            //pivot = -gamepad1.right_stick_x;
            //vertical = -gamepad1.left_stick_y;

            //Mecanum drive
            //Right_Front.setPower(pivot + (-vertical + horizontal));
            //Right_Back.setPower(pivot + (-vertical - horizontal));
            //Left_Back.setPower(pivot + (vertical - horizontal));
            //Left_Front.setPower(pivot + (vertical + horizontal));
            //waitForStart();


            //Code for arm

            while (opModeIsActive()) {
                PID.setPID(1.5,.0 ,.0);
                PID.setSetPoint(-900);
                PID.updatePID(Arm1.getCurrentPosition());
                if (gamepad1.a) {
                    PID.setSetPoint(-900);
                    PID.updatePID(Arm1.getCurrentPosition());
                    Arm1.setPower(1);
                    Arm1.setTargetPosition(PID.getSetPoint());
                    Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
                if (gamepad1.b) {


                    Arm1.setPower(.5);

                } else {
                    Arm1.setPower(0);
                }
                Arm1.getCurrentPosition();
                telemetry.addData("Arm Encode", Arm1.getCurrentPosition());
                telemetry.update();
            }

        }


        telemetry.addData("Arm Encode", Arm1.getCurrentPosition());
        telemetry.addData("Arm Inches", ticksToInches(Arm1.getCurrentPosition()));
        telemetry.addData("PID Result", PID.getResult());
        telemetry.update();



    }

    public double ticksToInches(double ticks) {
        double inches = ticks / 52;
        System.out.println(inches);
        return inches;
    }
    public double inchToTicks(double inches){
        double ticks = inches * 52;
        System.out.println(ticks);
        return ticks;
    }
}
//Arm.setTargetPosition(PID.getSetPoint());
//            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



