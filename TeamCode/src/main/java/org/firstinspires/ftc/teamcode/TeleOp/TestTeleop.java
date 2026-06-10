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

    private double flywheelSpeedR, flywheelsSpeedL;


    private int CurVol = 0;
    private int CurOpenSpeed = 0;
    private  int CurCloseSpeed = 0;
    private static final double OPEN_SERVO_POS = 0.25;

    private static final double INTAKE_POWER = -0.6;
    private static final double HELPER_POWER = 1;

    /// Mecanum driving
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;

    /// LimeLight vision
    LimeLightVision LimeLight = new LimeLightVision();
    Pose3D botPose;
    List<Double> AprilTagCoords;
    LLResult CamResult;

    private double LastGoodReadVel = 1600;


    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastRight = false;
    boolean lastLeft = false;


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

        flywheelSpeedR = hw.flywheelR.getVelocity();
        flywheelsSpeedL = hw.flywheelL.getVelocity();

        if(CamResult != null && CamResult.isValid()) {
            telemetry.addData("FB", AprilTagCoords.get(0));
            telemetry.addData("LR", AprilTagCoords.get(1));
            telemetry.addData("H", AprilTagCoords.get(2));
            telemetry.addData("DD", AprilTagCoords.get(3));
        }else{
            telemetry.addData("FB", 0);
            telemetry.addData("LR", 0);
            telemetry.addData("H", 0);
            telemetry.addData("DD", 0);
        }

        if (gamepad1.dpad_up && !lastUp) {
            CurVol += 50;
            setFlywheels(CurVol);
        }
        if (gamepad1.dpad_down && !lastDown) {
            CurVol -= 50;
            setFlywheels(CurVol);
        }

        if (gamepad1.dpad_right && !lastRight) {
            CurCloseSpeed += 50; // kP usually needs slightly larger steps than minPower

        }
        if (gamepad1.dpad_left && !lastLeft) {
            CurCloseSpeed -= 50;
        }

        if (gamepad1.left_bumper){
            double newRotate = drive.autoTarget(AprilTagCoords);
            drive.driveFieldRelative(forward, strafe, newRotate);
        }else {
            drive.driveFieldRelative(forward, strafe, rotate);
        }

        if (gamepad1.right_trigger_pressed){
            hw.flywheelR.setVelocity(CurVol);
            hw.flywheelL.setVelocity(CurVol);
            hw.intake.setPower(INTAKE_POWER);
            hw.helperMotor.setPower(HELPER_POWER);
        } else if (gamepad1.right_bumper){
            hw.flywheelR.setVelocity(0);
            hw.flywheelL.setVelocity(0);
            hw.intake.setPower(INTAKE_POWER);
            hw.helperMotor.setPower(HELPER_POWER);
        }else if (gamepad1.left_trigger_pressed){
            if (CamResult != null) {
                double vel = LimeLight.getVelocityCubicMath(AprilTagCoords.get(0));
                LastGoodReadVel = vel;
                hw.flywheelR.setVelocity(vel);
                hw.flywheelL.setVelocity(vel);
                hw.intake.setPower(INTAKE_POWER);
                hw.helperMotor.setPower(HELPER_POWER);
            }
            else{
                hw.flywheelR.setVelocity(LastGoodReadVel);
                hw.flywheelL.setVelocity(LastGoodReadVel);
                hw.intake.setPower(INTAKE_POWER);
                hw.helperMotor.setPower(HELPER_POWER);
            }
        } else {
            hw.flywheelR.setVelocity(0);
            hw.flywheelL.setVelocity(0);
            hw.intake.setPower(0);
            hw.helperMotor.setPower(0);
        }

        if (flywheelSpeedR >= CurOpenSpeed && flywheelsSpeedL >= CurOpenSpeed){
            hw.shooterServo.setPosition(OPEN_SERVO_POS);
        }else if (flywheelSpeedR <= CurCloseSpeed && flywheelsSpeedL <= CurCloseSpeed){
            hw.shooterServo.setPosition(0);
        }

        telemetry.addData("CurVol", CurVol);
        telemetry.addData("CurOpenSpeed", CurOpenSpeed);
        telemetry.addData("CurCloseSpeed", CurCloseSpeed);


        //Reset the IMU using the "Options" button
        if(gamepad1.optionsWasPressed()){
            hw.ResetImu();
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        LimeLight.StopVision();
    }

    private void setFlywheels(int speed){
        hw.flywheelL.setVelocity(speed);
        hw.flywheelR.setVelocity(speed);
    }
}
