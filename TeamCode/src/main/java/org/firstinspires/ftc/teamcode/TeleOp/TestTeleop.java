package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Vision.LimeLightVision;

import java.util.List;

@TeleOp(name = "==Test==")
public class TestTeleop extends OpMode {

    // Hardware setup
    Hardware hw = new Hardware();

    private double flywheelSpeedR, flywheelsSpeedL;

    // These will now be updated automatically by the cubic math equations
    private int CurVol = 0;
    private int CurOpenSpeed = 0;
    private int CurCloseSpeed = 0;

    private static final double OPEN_SERVO_POS = 0.25;
    private static final double INTAKE_POWER = -0.6;
    private static final double HELPER_POWER = 1;

    // Mecanum driving
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;

    // LimeLight vision
    LimeLightVision LimeLight = new LimeLightVision();
    Pose3D botPose;
    List<Double> AprilTagCoords;
    LLResult CamResult;
    public double orientation;

    private boolean UseAutoVel = true;
    private boolean ManualShootClose = true;

    private final double CloseShootDistance = 2.00;
    private final double FarShootDistance = 3.00;

    // Debounce tracking states
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
        // 1. Read default joystick inputs
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        hw.imu.update();

        CamResult = LimeLight.UpdateCamera(orientation);
        botPose = LimeLight.UpdateBotPos();
        AprilTagCoords = LimeLight.GetAprilTagCoords();

        flywheelSpeedR = hw.flywheelR.getVelocity();
        flywheelsSpeedL = hw.flywheelL.getVelocity();

        CheckShootingMode();
        CheckManualShootingDistance();

        // 4. INTEGRATION: Automatically calculate outputs if Auto Mode is active
        if (UseAutoVel && CamResult != null && CamResult.isValid() && AprilTagCoords != null) {
            double rawDistance = AprilTagCoords.get(0);
            // Constrain math range between 1.5m and 3.0m to prevent curve runaway
            double x = Math.max(1.5, Math.min(3.0, rawDistance));

            // Execute the precise cubic target calculations derived from your lookup table
            CurVol        = (int) ((-133.3333 * Math.pow(x, 3)) + (900.0 * Math.pow(x, 2)) - (1816.6667 * x) + 2600.0);
            CurOpenSpeed  = (int) ((-66.6667 * Math.pow(x, 3)) + (400.0 * Math.pow(x, 2)) - (583.3333 * x) + 1550.0);
            CurCloseSpeed = (int) ((66.6667 * Math.pow(x, 3)) - (400.0 * Math.pow(x, 2)) + (983.3333 * x) + 300.0);
        }

        // 5. Manual overrides (D-pad & bumpers can still adjust values when UseAutoVel is false)
        if (!UseAutoVel) {
            if (gamepad1.dpad_up && !lastUp)        CurVol += 50;
            if (gamepad1.dpad_down && !lastDown)    CurVol -= 50;
            if (gamepad1.dpad_right && !lastRight)  CurCloseSpeed += 50;
            if (gamepad1.dpad_left && !lastLeft)    CurCloseSpeed -= 50;
            if (gamepad1.left_bumper)               CurOpenSpeed += 50;
            if (gamepad1.circle)                    CurOpenSpeed -= 50;
        }

        lastUp    = gamepad1.dpad_up;
        lastDown  = gamepad1.dpad_down;
        lastRight = gamepad1.dpad_right;
        lastLeft  = gamepad1.dpad_left;

        // 6. Flywheel / Intake execution
        // Changed to float trigger check since standard gamepads return scalar value
        if (gamepad1.right_trigger > 0.2) {
            hw.intake.setPower(INTAKE_POWER);
            hw.helperMotor.setPower(HELPER_POWER);
            setFlywheels(CurVol);
        } else if (gamepad1.right_bumper) {
            setFlywheels(0);
            hw.intake.setPower(INTAKE_POWER);
            hw.helperMotor.setPower(HELPER_POWER);
        } else {
            setFlywheels(0);
            hw.intake.setPower(0);
            hw.helperMotor.setPower(0);
        }

        // 7. Servo controller trigger logic
        if (flywheelSpeedR >= CurOpenSpeed && flywheelsSpeedL >= CurOpenSpeed) {
            hw.shooterServo.setPosition(OPEN_SERVO_POS);
        } else if (flywheelSpeedR <= CurCloseSpeed && flywheelsSpeedL <= CurCloseSpeed) {
            hw.shooterServo.setPosition(0);
        }

        // 8. Handle Drivetrain logic and process automated steering corrections
        CheckAutoAim();

        // 9. Display diagnostics
        if (CamResult != null && CamResult.isValid()) {
            telemetry.addData("FB (Dist)", AprilTagCoords.get(0));
            telemetry.addData("LR (Offset)", AprilTagCoords.get(1));
        } else {
            telemetry.addLine("--- NO TARGET LOCKED ---");
        }
        telemetry.addData("Target Velocity", CurVol);
        telemetry.addData("Trigger Open threshold", CurOpenSpeed);
        telemetry.addData("Trigger Close threshold", CurCloseSpeed);
        telemetry.addData("Automation Active", UseAutoVel);
        telemetry.update();

        if (gamepad1.optionsWasPressed()) {
            hw.ResetImu();
        }
    }

    @Override
    public void stop() {
        LimeLight.StopVision();
    }

    private void setFlywheels(int speed) {
        hw.flywheelL.setVelocity(speed);
        hw.flywheelR.setVelocity(speed);
    }

    private void CheckShootingMode() {
        if (gamepad1.left_stick_button) {
            UseAutoVel = true;
        } else if (gamepad1.right_stick_button) {
            UseAutoVel = false;
        }
    }

    private void CheckManualShootingDistance() {
        if (UseAutoVel) return;

        if (gamepad1.triangle) {
            ManualShootClose = false;
        } else if (gamepad1.cross) {
            ManualShootClose = true;
        }
    }

    private void CheckAutoAim() {
        // If holding SQUARE, let tracking calculations override manual driver commands
        if (gamepad1.square) {
            if (UseAutoVel) {
                if (CamResult != null && CamResult.isValid() && AprilTagCoords != null) {
                    rotate = drive.autoTarget(AprilTagCoords);
                } else {
                    rotate = gamepad1.right_stick_x; // Fallback to manual control if target is lost
                }
                // Send manual translation controls mixed with automated camera alignment steering
                drive.driveFieldRelative(forward, strafe, rotate);
            }
            else {
                // In manual mode, hand full coordinate control over to alignment function
                if (ManualShootClose) {
                    drive.AutoAimManualMode(AprilTagCoords, CloseShootDistance);
                } else {
                    drive.AutoAimManualMode(AprilTagCoords, FarShootDistance);
                }
            }
        }
        else {
            // Standard manual field-centric navigation fallback
            drive.driveFieldRelative(forward, strafe, rotate);
        }
    }
}