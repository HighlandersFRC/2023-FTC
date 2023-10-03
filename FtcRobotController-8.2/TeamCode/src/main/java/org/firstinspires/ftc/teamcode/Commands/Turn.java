package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PID1;

public class Turn extends Command{
    PID1 PID = new PID1(0.03, 0.0, 0.0);
    public DcMotor Left_Back;
    public DcMotor Right_Back;
    public DcMotor Left_Front;
    public DcMotor Right_Front;
    public double targetAngle;
    public IMU imu;
    public double currentPos;
    public Turn(HardwareMap hardwareMap, double targetAngle){
        this.targetAngle = targetAngle;
        PID.setSetPoint(targetAngle);
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }
    public void start() {
        imu.resetYaw();
 }
    public void execute() {

        currentPos = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        PID.updatePID(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        double power = PID.getResult();
        Left_Front.setPower(power);
        Left_Back.setPower(power);
        Right_Front.setPower(power);
        Right_Back.setPower(power);
    }

        public void end() {

    }

    public boolean isFinished() {
        if (PID.getSetPoint() - 1 <= imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) && imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) <=  PID.getSetPoint() + 1 && PID.getResult() < 0.05) {
            return true;
        }
        return false;
    }
}