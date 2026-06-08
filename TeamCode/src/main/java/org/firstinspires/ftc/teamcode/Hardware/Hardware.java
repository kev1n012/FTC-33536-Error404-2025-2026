package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class Hardware {

    public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotorEx flywheelL, flywheelR, intake, helperMotor;

    public Servo shooterServo;
    public IMU imu;
    public Limelight3A limelight;
    public NormalizedColorSensor colorSensorLeft, colorSensorRight, colorSensorMiddle;

    public void init(HardwareMap hardwareMap){

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "back_right");

        flywheelL = hardwareMap.get(DcMotorEx.class, "flywheelL");
        flywheelR = hardwareMap.get(DcMotorEx.class, "flywheelR");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooterServo = hardwareMap.get(Servo.class, "shooter_servo");

        helperMotor = hardwareMap.get(DcMotorEx.class, "helper_motor");


        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        flywheelR.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelL.setDirection(DcMotorEx.Direction.REVERSE);

        intake.setDirection(DcMotorEx.Direction.REVERSE);
        helperMotor.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        //TODO fill in the correct name
        limelight = hardwareMap.get(Limelight3A.class, "PlaceHolder");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }
    public void ResetImu(){
        imu.resetYaw();
    }
}
