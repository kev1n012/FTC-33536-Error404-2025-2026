package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Vision.LimeLightVision;


@TeleOp(name = "ImprovedBackupTeleOp")
public class ImprovedBackupTeleOp extends OpMode {

    Hardware hw = new Hardware();

    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;


    LimeLightVision LimeLight = new LimeLightVision();
    Pose3D botPose;
    LLResult result;




    private double flywheelSpeedR = 0;
    private double flywheelsSpeedL = 0;

    private static final double OPEN_SERVO_POS = 0.25;

    //TODO change from RPM to velocity
    private final int DESIRED_FLYWHEEL_SPEED = 3900;
    private final int CLOSE_SHOOTER_SERVO = 1500;

    private static final double INTAKE_POWER = -0.3;

    private static double SHOOTER_POWER = 0.80;


    @Override
    public void init() {
        hw.init(hardwareMap);
        drive.init(hw);
        LimeLight.init(hw);

        LimeLight.ChangePipeline(0);
        hw.shooterServo.setPosition(0);
        telemetry.addData("Status", "Fully Initialized");
    }

    @Override
    public void start() {
        LimeLight.StartVision();
    }

    @Override
    public void loop() {

        flywheelSpeedR = hw.flywheelR.getVelocity();
        flywheelsSpeedL = hw.flywheelL.getVelocity();

        updateShooterAndIntake();




        if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
            hw.ResetImu();
        }

        telemetry.update();
    }

    private void updateShooterAndIntake() {
        //TODO Make this a state machine
        if (gamepad1.right_bumper) {
            hw.intake.setPower(INTAKE_POWER);
            hw.helperMotor.setPower(0.5);
            hw.flywheelL.setPower(0.0);
            hw.flywheelR.setPower(0.0);
            hw.shooterServo.setPosition(0);
        } else if (gamepad1.right_trigger >= 0.2f) {
            hw.intake.setPower(INTAKE_POWER);
            hw.helperMotor.setPower(0.5);
            hw.flywheelL.setPower(SHOOTER_POWER);
            hw.flywheelR.setPower(SHOOTER_POWER);
            if (flywheelsSpeedL > DESIRED_FLYWHEEL_SPEED && flywheelSpeedR > DESIRED_FLYWHEEL_SPEED) {
                hw.shooterServo.setPosition(OPEN_SERVO_POS);
            }
        } else if (gamepad1.left_trigger >= 0.5f) {
            hw.intake.setPower(-INTAKE_POWER);
            hw.helperMotor.setPower(-0.5);
            hw.flywheelL.setPower(-SHOOTER_POWER);
            hw.flywheelR.setPower(-SHOOTER_POWER);
            hw.shooterServo.setPosition(0.25);
        } else {
            hw.intake.setPower(0.0);
            hw.helperMotor.setPower(0.0);
            hw.flywheelL.setPower(0.0);
            hw.flywheelR.setPower(0.0);
            if (flywheelsSpeedL > CLOSE_SHOOTER_SERVO && flywheelSpeedR > CLOSE_SHOOTER_SERVO) {
                hw.shooterServo.setPosition(0);
            }
        }
    }

}