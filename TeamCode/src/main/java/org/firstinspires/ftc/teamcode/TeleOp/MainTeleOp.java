package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Vision.LimeLightVision;

public class MainTeleOp extends OpMode {
    Hardware hw = new Hardware();
    MecanumDrive drive = new MecanumDrive();
    LimeLightVision vision = new LimeLightVision();
    double forward, strafe, rotate;

    Pose3D botPose;

    @Override
    public void init() {
        hw.init(hardwareMap);
        drive.init(hw);
        vision.init(hw);

        //TODO set the correct pipeline
        vision.ChangePipeline(0);
    }

    @Override
    public void start() {
        vision.StartVision();
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        botPose = vision.updateVision();

        drive.driveFieldRelative(forward, strafe, rotate);
    }
}
