package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class test1 extends LinearOpMode {

    private DcMotor Left_Front;
    private DcMotor Right_Front;
    private DcMotor Left_Back;
    private DcMotor Right_Back;
    private DcMotor Arm;
    @Override

    public void runOpMode() {
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");
        Arm = hardwareMap.dcMotor.get("Arm");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

                double vertical;
                double horizontal;
                double pivot;
                boolean lift;
                boolean down;
            // The codes to define the wheels way
                vertical = gamepad1.left_stick_y;
                horizontal = gamepad1.left_stick_x;
                pivot = gamepad1.right_stick_x;

                //The arm is using the y button to go up
                lift = gamepad1.y;

                //The arm is using the y button to go up
                down = gamepad1.a;

            //The code for the Mecanum drive
                Right_Front.setPower(pivot+( vertical - horizontal));
                Right_Back.setPower(pivot+( vertical + horizontal));
                Left_Front.setPower(pivot+( -vertical - horizontal));
                Left_Back.setPower(pivot+( -vertical + horizontal));


                //The code where the arm go up and down

                if (lift=true) {
                    Arm.setPower(1);



                }
                if (down=true){
                    Arm.setPower(-1);


                }
        }
    }
}
