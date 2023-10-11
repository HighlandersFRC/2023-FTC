package org.firstinspires.ftc.teamcode.Commands;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Angle;

public class Rotate extends Command {
    private DcMotor Left_Back;
    private DcMotor Right_Back;
    private DcMotor Left_Front;
    private DcMotor Right_Front;
    private BNO055IMU imu;
    double angle;
    double state;
    double integralSum = 0;
    double Kp = 0.1;
    double Ki = 0;
    double Kd = 0;
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
    public Rotate(HardwareMap hardwareMap, double angle, double state){
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }
    public void start(){

    }
    public void execute(){
double power = PIDControl(referenceAngle, imu.getAngularOrientation().firstAngle);
        Left_Front.setPower(power);
        Left_Back.setPower(power);
        Right_Front.setPower(-power);
        Right_Back.setPower(-power);
    }
    public void end(){
        Left_Front.setPower(0);
        Right_Front.setPower(0);
        Left_Back.setPower(0);
        Right_Back.setPower(0);
    }

    public boolean isFinished() {
        return false;
    }
}
