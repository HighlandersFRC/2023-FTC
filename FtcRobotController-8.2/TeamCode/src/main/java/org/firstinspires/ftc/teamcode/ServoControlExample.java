package org.firstinspires.ftc.teamcode;// Import necessary classes from the FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Define your custom OpMode class
@TeleOp(name = "ServoControlExample", group = "FTC")
public class ServoControlExample extends LinearOpMode {

    // Declare servo and other variables

    private ElapsedTime runtime = new ElapsedTime();
    Servo servo1;

    @Override
    public void runOpMode() {

        // Initialize servo
        servo1 = hardwareMap.get(Servo.class, "servo1");

        // Wait for the start button to be pressed on the driver station
        waitForStart();

        // Main loop
        while (opModeIsActive()) {

            // Control the servo position


            {

                    // Add telemetry to display servo position on the driver station
                while (opModeIsActive()) {
                    if(gamepad1.x) {
                        double targetPosition = 1;  // Adjust this value as needed
                        servo1.setPosition(1.0);
                    }
                    telemetry.addData("Servo Position", servo1.getPosition());
                    telemetry.addData("Status", "Initialized");
                    servo1 = hardwareMap.servo.get("servo1");
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.update();

                    // Sleep for a short period to avoid continuous servo movement
                    sleep(1);  // Adjust the sleep time as needed
                }
                }
            }
        }
    }
