package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Vision.LimeLightVision;
import java.util.List;


@TeleOp(name = "ImprovedBackupTeleOp")
public class ImprovedBackupTeleOp extends OpMode {

    Hardware hw = new Hardware();

    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;


    LimeLightVision LimeLight = new LimeLightVision();
    Pose3D botPose;
    List<Double> AprilTagCoords;
    LLResult CamResult;

    enum RobotState{
        STATE_SHOOT,
        STATE_INTAKE,
        STATE_REJECT,
        STATE_IDLE
    }
    RobotState currentRobotState = RobotState.STATE_IDLE;


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
        CamResult = LimeLight.UpdateCamera();
        botPose = LimeLight.UpdateBotPos();
        //AprilTagCoords = LimeLight.GetAprilTagCoords();

        flywheelSpeedR = hw.flywheelR.getVelocity();
        flywheelsSpeedL = hw.flywheelL.getVelocity();

        updateDriveInputs();
        UpdateState();
        updateShooterAndIntake();

        if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
            hw.ResetImu();
        }
        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Rotate", rotate);
        drive.driveFieldRelative(forward, strafe, rotate);

        telemetry.update();
    }
    private void updateDriveInputs(){
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
    }
    private void updateShooterAndIntake() {
        switch (currentRobotState){
            case STATE_IDLE:
                hw.intake.setPower(0.0);
                hw.helperMotor.setPower(0.0);
                hw.flywheelL.setPower(0.0);
                hw.flywheelR.setPower(0.0);
                if (flywheelsSpeedL > CLOSE_SHOOTER_SERVO && flywheelSpeedR > CLOSE_SHOOTER_SERVO) {
                    hw.shooterServo.setPosition(0);
                }
                break;

            case STATE_SHOOT:
                hw.intake.setPower(INTAKE_POWER);
                hw.helperMotor.setPower(0.5);
                hw.flywheelL.setPower(SHOOTER_POWER);
                hw.flywheelR.setPower(SHOOTER_POWER);
                if (flywheelsSpeedL > DESIRED_FLYWHEEL_SPEED && flywheelSpeedR > DESIRED_FLYWHEEL_SPEED) {
                    hw.shooterServo.setPosition(OPEN_SERVO_POS);
                }
                break;

            case STATE_INTAKE:
                hw.intake.setPower(INTAKE_POWER);
                hw.helperMotor.setPower(0.5);
                hw.flywheelL.setPower(0.0);
                hw.flywheelR.setPower(0.0);
                hw.shooterServo.setPosition(0);
                break;

            case STATE_REJECT:
                hw.intake.setPower(-INTAKE_POWER);
                hw.helperMotor.setPower(-0.5);
                hw.flywheelL.setPower(-SHOOTER_POWER);
                hw.flywheelR.setPower(-SHOOTER_POWER);
                hw.shooterServo.setPosition(0.25);
                break;
        }
    }

    private void UpdateState(){
        if(gamepad1.right_trigger_pressed){
            currentRobotState = RobotState.STATE_SHOOT;
        } else if (gamepad1.left_trigger_pressed){
            currentRobotState = RobotState.STATE_REJECT;
        } else if (gamepad1.right_bumper){
            currentRobotState = RobotState.STATE_INTAKE;
        } else {
            currentRobotState = RobotState.STATE_IDLE;
        }
    }

}