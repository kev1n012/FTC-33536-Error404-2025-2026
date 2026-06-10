package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Vision.ColorSensorVision;
import org.firstinspires.ftc.teamcode.Vision.LimeLightVision;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "==Test==")

public class TestTeleop extends OpMode {
    /// Hardware setup
    Hardware hw = new Hardware();

    /// Mecanum driving
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;

    /// LimeLight vision
    LimeLightVision LimeLight = new LimeLightVision();
    Pose3D botPose;
    List<Double> AprilTagCoords;
    LLResult CamResult;


    @Override
    public void init() {
        hw.init(hardwareMap);
        drive.init(hw);
        LimeLight.init(hw);
        LimeLight.ChangePipeline(9);
        LimeLight.StartVision();
        telemetry.addLine("Init complete");
    }


    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        CamResult = LimeLight.UpdateCamera();
        botPose = LimeLight.UpdateBotPos();
        AprilTagCoords = LimeLight.GetAprilTagCoords();

        if(CamResult != null && CamResult.isValid()) {
            telemetry.addData("FB", AprilTagCoords.get(0));
            telemetry.addData("LR", AprilTagCoords.get(1));
            telemetry.addData("H", AprilTagCoords.get(2));
            telemetry.addData("DD", AprilTagCoords.get(3));
        }

        if (gamepad1.left_bumper){
            drive.autoTarget(AprilTagCoords);
        }else {
            drive.driveFieldRelative(forward, strafe, rotate);
        }

        //Reset the IMU using the "Options" button
        if(gamepad1.optionsWasPressed()){
            hw.ResetImu();
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        LimeLight.StopVision();
    }
}
