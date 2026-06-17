package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Vision.LimeLightVision;
import java.util.List;

@TeleOp(name = "TeleOp BLUE")
public class TeleOpBlue extends OpMode {

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
        STATE_IDLE,
        STATE_STOP
    }
    RobotState currentRobotState = RobotState.STATE_IDLE;

    enum ShootingMethod{
        STATE_AUTO,
        STATE_MANUAL
    }
    ShootingMethod currentShootingMethod = ShootingMethod.STATE_AUTO;

    enum ManualShootingDistance{
        DISTANCE_CLOSE,
        DISTANCE_FAR
    }
    ManualShootingDistance currentShootingDistance = ManualShootingDistance.DISTANCE_CLOSE;
    private final double CloseShootDistance = 2.00;
    private final double FarShootDistance = 3.00;
    private double flywheelSpeedR, flywheelsSpeedL;
    private int CurVol = 0;
    private int CurOpenSpeed = 0;
    private int CurCloseSpeed = 0;
    private static double FlywheelIdleVelocity = 800;
    private static final double OPEN_SERVO_POS = 0.25;
    private static final double INTAKE_POWER = -0.6;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;



    @Override
    public void init() {
        hw.init(hardwareMap);
        drive.init(hw);
        LimeLight.init(hw);
        LimeLight.StartVision();
        LimeLight.ChangePipeline(1);
        hw.shooterServo.setPosition(0);
        telemetry.addData("Status", "Fully Initialized");
    }

    @Override
    public void loop() {
        hw.imu.update();

        CamResult = LimeLight.UpdateCamera();
        botPose = LimeLight.UpdateBotPos();
        AprilTagCoords = LimeLight.GetAprilTagCoords();

        flywheelSpeedR = hw.flywheelR.getVelocity();
        flywheelsSpeedL = hw.flywheelL.getVelocity();

        //Functions with user input
        UpdateState();
        updateDriveInputs();

        //Update values
        UpdateCurrentFlywheelVel();

        //Setting hardware
        ManageServo();
        SetShooterValues();
        AutoAim();
        updateShooterAndIntake();

        if (gamepad1.options) {
            hw.ResetImu();
        }

        if (!gamepad1.square){ drive.driveFieldRelative(forward, strafe, rotate); }

        GameInfoTelemetry();

        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        currentRobotState = RobotState.STATE_STOP;
        LimeLight.StopVision();
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
                hw.setFlywheels(FlywheelIdleVelocity);
                break;

            case STATE_SHOOT:
                hw.intake.setPower(INTAKE_POWER);
                hw.helperMotor.setPower(1);
                hw.setFlywheels(CurVol);
                break;

            case STATE_INTAKE:
                hw.intake.setPower(INTAKE_POWER);
                hw.helperMotor.setPower(1);
                hw.shooterServo.setPosition(0);
                hw.setFlywheels(FlywheelIdleVelocity);
                break;

            case STATE_REJECT:
                hw.intake.setPower(-INTAKE_POWER);
                hw.helperMotor.setPower(-1);
                hw.setFlywheels(-1700);
                hw.shooterServo.setPosition(0.25);
                break;
            case STATE_STOP:
                hw.setFlywheels(0);
                hw.intake.setPower(0);
                hw.helperMotor.setPower(0);
                hw.shooterServo.setPosition(0);
        }
    }
    private void ManageServo(){
        if (currentRobotState == RobotState.STATE_REJECT){return;}
        if (currentRobotState != RobotState.STATE_SHOOT && currentRobotState != RobotState.STATE_IDLE){ hw.shooterServo.setPosition(0); return;}

        if (flywheelSpeedR < CurCloseSpeed && flywheelsSpeedL < CurCloseSpeed){
            hw.shooterServo.setPosition(0);
        }else if (flywheelsSpeedL > CurOpenSpeed && flywheelSpeedR > CurOpenSpeed){
            hw.shooterServo.setPosition(OPEN_SERVO_POS);
        }
    }
    private void UpdateState(){
        if(gamepad1.right_trigger >= 0.2){
            currentRobotState = RobotState.STATE_SHOOT;
        } else if (gamepad1.left_trigger >= 0.2){
            currentRobotState = RobotState.STATE_REJECT;
        } else if (gamepad1.right_bumper){
            currentRobotState = RobotState.STATE_INTAKE;
        } else {
            currentRobotState = RobotState.STATE_IDLE;
        }

        if (currentShootingMethod == ShootingMethod.STATE_MANUAL) {
            if (gamepad1.dpad_down && !lastDpadDown) {
                currentShootingDistance = ManualShootingDistance.DISTANCE_CLOSE;
            } else if (gamepad1.dpad_up && !lastDpadUp) {
                currentShootingDistance = ManualShootingDistance.DISTANCE_FAR;
            }
        }

        if (gamepad1.dpad_left && !lastDpadLeft) {
            currentShootingMethod = ShootingMethod.STATE_AUTO;
        } else if (gamepad1.dpad_right && !lastDpadRight) {
            currentShootingMethod = ShootingMethod.STATE_MANUAL;
        }

        lastDpadDown = gamepad1.dpad_down;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
    }
    private void UpdateCurrentFlywheelVel(){
        flywheelSpeedR = hw.flywheelR.getVelocity();
        flywheelsSpeedL = hw.flywheelL.getVelocity();
    }
    private void SetShooterValues(){
        if (currentShootingMethod == ShootingMethod.STATE_AUTO && CamResult != null && CamResult.isValid() && AprilTagCoords != null) {
            double rawDistance = AprilTagCoords.get(0);

            LimeLightVision.ShotPreset activeShot = LimeLight.getInterpolatedShot(rawDistance);

            CurVol        = (int) activeShot.velocity;
            CurOpenSpeed  = (int) activeShot.servoOpen;
            CurCloseSpeed = (int) activeShot.servoClose;
        }else if (currentShootingMethod == ShootingMethod.STATE_MANUAL){

            switch (currentShootingDistance){
                case DISTANCE_CLOSE:
                    CurVol = 1500;
                    CurOpenSpeed = 1450;
                    CurCloseSpeed = 1200;
                    break;
                case DISTANCE_FAR:
                    CurVol = 1650;
                    CurOpenSpeed = 1600;
                    CurCloseSpeed = 1450;
                    break;
            }
        }
    }
    private void AutoAim(){
        if (!gamepad1.square){ return; }

        if (currentShootingMethod == ShootingMethod.STATE_AUTO) {
            if (CamResult != null && CamResult.isValid() && AprilTagCoords != null) {
                rotate = drive.autoTarget(AprilTagCoords);
            } else {
                rotate = gamepad1.right_stick_x;
            }
            drive.driveFieldRelative(forward, strafe, rotate);
        }
        else if (currentShootingMethod == ShootingMethod.STATE_MANUAL){
            if (currentShootingDistance == ManualShootingDistance.DISTANCE_CLOSE) {
                drive.AutoAimManualMode(AprilTagCoords, CloseShootDistance);
            } else {
                drive.AutoAimManualMode(AprilTagCoords, FarShootDistance);
            }
        }
    }
    private void GameInfoTelemetry(){
        telemetry.addData("CurVol", CurVol);
        telemetry.addData("CurOpenSpeed", CurOpenSpeed);
        telemetry.addData("CurCloseSpeed", CurCloseSpeed);

        if (CamResult != null && CamResult.isValid() && AprilTagCoords != null && !AprilTagCoords.isEmpty()) {
            telemetry.addData("Target Distance", "%.2f M", AprilTagCoords.get(0));
        }else {
            telemetry.addData("Target Distance", "N/A");
        }
    }
}
