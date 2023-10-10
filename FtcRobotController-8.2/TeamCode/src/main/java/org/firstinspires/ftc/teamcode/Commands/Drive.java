package org.firstinspires.ftc.teamcode.Commands;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive extends Command {
    private DcMotor Left_Back;
    private DcMotor Right_Back;
    private DcMotor Left_Front;
    private DcMotor Right_Front;
    public IMU imu;
    public double currentPos;
    double power;
    double firstAngle;
    double deviation;
    double leftPower;
    double rightPower;
    long distance;
    public Drive(HardwareMap hardwareMap, double speed, long distance){
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        this.power = speed - 0.05;
        this.distance = distance;
    }
    public void start(){
        firstAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void execute(){
        deviation = (Math.abs(firstAngle) - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) / 180;

        if (deviation > 0){
            leftPower = power + deviation;
            rightPower = power - deviation;
            Left_Front.setPower(leftPower);
            Right_Front.setPower(rightPower);
            Left_Back.setPower(leftPower);
            Right_Back.setPower(rightPower);
        }
        else{
            leftPower = power - deviation;
            rightPower = power + deviation;
            Left_Front.setPower(leftPower);
            Left_Back.setPower(leftPower);
            Right_Back.setPower(-rightPower);
        }
        Left_Front.setPower(leftPower);
        Right_Front.setPower(-rightPower);
        Left_Back.setPower(leftPower);
        Right_Back.setPower(-rightPower);
        currentPos = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void end(){
        Left_Front.setPower(0);
        Right_Front.setPower(0);
        Left_Back.setPower(0);
        Right_Back.setPower(0);
    }

    public boolean isFinished() {
       // if (System.currentTimeMillis()){
       // return true;
       // }
      //  else {
            return false;
      //  }
    }
}
