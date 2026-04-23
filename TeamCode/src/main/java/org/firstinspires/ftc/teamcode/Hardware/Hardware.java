package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class Hardware {

    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public IMU imu;
    public Limelight3A limelight;
    public NormalizedColorSensor colorSensorLeft, colorSensorRight, colorSensorMiddle;

    public void init(HardwareMap hardwareMap){

        //TODO change the motor names
        frontLeftMotor = hardwareMap.get(DcMotor.class, "PlaceHolder");
        frontRightMotor = hardwareMap.get(DcMotor.class, "PlaceHolder");
        backLeftMotor = hardwareMap.get(DcMotor.class, "PlaceHolder");
        backRightMotor = hardwareMap.get(DcMotor.class, "PlaceHolder");

        //TODO set the correct directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        //TODO set the correct names
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "PlaceHolder");
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, "PlaceHolder");
        colorSensorMiddle = hardwareMap.get(NormalizedColorSensor.class, "PlaceHolder");

        //TODO tune the gain for each color sensor
        colorSensorLeft.setGain(0);
        colorSensorRight.setGain(0);
        colorSensorMiddle.setGain(0);

        //TODO fill in the correct name
        limelight = hardwareMap.get(Limelight3A.class, "PlaceHolder");

        //TODO set the correct orientation
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void ResetImu(){
        imu.resetYaw();
    }
}
