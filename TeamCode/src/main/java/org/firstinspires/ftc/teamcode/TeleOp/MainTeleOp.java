package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Vision.ColorSensorVision;
import org.firstinspires.ftc.teamcode.Vision.LimeLightVision;

public class MainTeleOp extends OpMode {
    //Hardware setup
    Hardware hw = new Hardware();

    //Mecanum driving
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;

    // LimeLight vision
    LimeLightVision LimeLight = new LimeLightVision();
    Pose3D botPose;
    LLResult result;

    //Color sensors
    ColorSensorVision ColorSensor = new ColorSensorVision();
    ColorSensorVision.DetectableColors detectedColorRightSensor;
    ColorSensorVision.DetectableColors detectedColorLeftSensor;
    ColorSensorVision.DetectableColors detectedColorMiddleSensor;




    @Override
    public void init() {
        hw.init(hardwareMap);
        drive.init(hw);
        LimeLight.init(hw);
        ColorSensor.init(hw);

        //TODO set the correct pipeline
        LimeLight.ChangePipeline(0);
    }

    @Override
    public void start() {
        //If the camera falls behind move this to init()
        LimeLight.StartVision();
    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // Where the LimeLight thinks the robot is
        botPose = LimeLight.UpdateBotPos();

        // The distance and angle to the april tag
        result = LimeLight.GetResults();

        detectedColorLeftSensor = ColorSensor.getDetectedColor(hw.colorSensorLeft, telemetry);
        detectedColorRightSensor = ColorSensor.getDetectedColor(hw.colorSensorRight, telemetry);
        detectedColorMiddleSensor = ColorSensor.getDetectedColor(hw.colorSensorMiddle, telemetry);

        drive.driveFieldRelative(forward, strafe, rotate);

        //Reset the IMU using the "Options" button
        if(gamepad1.optionsWasPressed()){
            hw.ResetImu();
        }
    }
}
