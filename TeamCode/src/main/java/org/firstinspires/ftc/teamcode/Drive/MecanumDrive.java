package org.firstinspires.ftc.teamcode.Drive;

import android.graphics.HardwareRenderer;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class MecanumDrive {

    //Make an empty class for the hardware
    private Hardware hw;

    public void init(Hardware hardware){
        //Fill the empty "hw" with the actual hardware
        this.hw = hardware;
    }
    public void drive(double forward, double strafe, double rotate){
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe  + rotate;
        double  backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        hw.frontLeftMotor.setPower(frontLeftPower / maxPower);
        hw.frontRightMotor.setPower(frontRightPower / maxPower);
        hw.backLeftMotor.setPower(backLeftPower / maxPower);
        hw.backRightMotor.setPower(backRightPower / maxPower);
    }

    public void driveFieldRelative(double forward, double strafe, double rotate){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newForward, rotate);
    }
}
