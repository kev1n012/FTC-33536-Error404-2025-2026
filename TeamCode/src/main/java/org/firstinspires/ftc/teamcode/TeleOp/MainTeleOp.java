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




    @Override
    public void init() {
        hw.init(hardwareMap);
        drive.init(hw);
        LimeLight.init(hw);

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
        //FIXME Update with new method (TestTelOp)
        //result = LimeLight.UpdateCamera();

        drive.driveFieldRelative(forward, strafe, rotate);

        //Reset the IMU using the "Options" button
        if(gamepad1.optionsWasPressed()){
            hw.ResetImu();
        }
    }
}
