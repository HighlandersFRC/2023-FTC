package org.firstinspires.ftc.teamcode;

        import android.os.PowerManager;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test_Servo", group="Linear OpMode")
//@Disabled
public class Test_Servo extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Servo servo;
    double servoPosition = 0.0;

    @Override
    public void runOpMode() {
        waitForStart();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servo = hardwareMap.servo.get("servo");
        servo.setPosition(servoPosition);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Position", servoPosition);
            telemetry.update();

            if (gamepad1.b)
            {
                servoPosition = 0.7843;
                servo.setPosition(servoPosition);


            }
            else
            {
                servoPosition = 0.1765;
                servo.setPosition(servoPosition);

            }
        }
    }
}
