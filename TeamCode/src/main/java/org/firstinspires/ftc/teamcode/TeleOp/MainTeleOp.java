package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class MainTeleOp extends OpMode {
    Hardware hardware = new Hardware();
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        drive.init(hardware);
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward, strafe, rotate);
    }
}
