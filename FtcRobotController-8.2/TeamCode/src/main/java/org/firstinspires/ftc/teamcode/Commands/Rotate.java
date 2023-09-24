package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PID1;

public class Rotate extends Command {
    private DcMotor Left_Back;
    private DcMotor Right_Back;
    private DcMotor Left_Front;
    private DcMotor Right_Front;
    private IMU imu;
    double angle;
    double state;
    double integralSum = 0;
    double Kp = 0.7;
    double Ki = 0.1;
    double Kd = 0.4;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    double referenceAngle = Math.toRadians(90);
    public double angleWrap(double radians){
        while (radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }


    public Rotate(HardwareMap hardwareMap, double angle, double state){
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        this.imu = imu;
        this.angle = referenceAngle;
        this.state = state;
    }
    public double PIDControl(double reference, double state){
        this.angle = reference;
        this.state = state;
        double error = reference - state;
        integralSum += error + timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error + Kp) + (derivative + Kd) + (integralSum + Ki);
        return output;
    }
    public void start(){
    imu.resetYaw();
    }
    public void execute(){

double power = PIDControl(referenceAngle, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        Left_Front.setPower(power);
        Left_Back.setPower(power);
        Right_Front.setPower(power);
        Right_Back.setPower(power);
    }
    public void end(){

    }

    public boolean isFinished() {
        return false;
    }
}
