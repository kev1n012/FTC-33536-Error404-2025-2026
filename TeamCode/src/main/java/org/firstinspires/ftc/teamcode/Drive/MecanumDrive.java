package org.firstinspires.ftc.teamcode.Drive;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import java.util.List;

public class MecanumDrive {

    // Make an empty class for the hardware
    private Hardware hw;

    // Tuned constants for smooth tracking
    private final double kP_Turn = 1.3;
    private final double kP_Drive = 0.8;       // Lowered from 1.3 to stop the aggressive forward/backward shaking
    private final double kP_Strafe = 1.0;      // Adjusted for cleaner sideways sliding
    private final double minPower = 0.04;      // Smooth floor to overcome friction
    private final double LR_DEADZONE_METERS = 0.02; // Slightly tightened deadzones
    private final double FB_DEADZONE_METERS = 0.02;

    public void init(Hardware hardware){
        this.hw = hardware;
    }

    public void drive(double forward, double strafe, double rotate){
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        hw.frontLeftMotor.setPower(frontLeftPower / maxPower);
        hw.frontRightMotor.setPower(frontRightPower / maxPower);
        hw.backLeftMotor.setPower(backLeftPower / maxPower);
        hw.backRightMotor.setPower(backRightPower / maxPower);
    }

    /**
     * Field-relative manual driver control.
     */
    public void driveFieldRelative(double forward, double strafe, double rotate){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        double robotHeadingRadians = Math.toRadians(hw.GetOrientation());
        theta = AngleUnit.normalizeRadians(theta - robotHeadingRadians);

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

    /**
     * Automatically rotates the robot to center on the AprilTag.
     */
    public double autoTarget(List<Double> aprilTagCoords) {
        if (aprilTagCoords == null || aprilTagCoords.size() < 2) {
            return 0.0;
        }

        double distanceLR = aprilTagCoords.get(1);

        if (Math.abs(distanceLR) <= LR_DEADZONE_METERS) {
            return 0.0;
        }

        // Camera X inversion applied for correct clockwise/counterclockwise reaction
        double rotatePower = -distanceLR * kP_Turn;

        if (Math.abs(rotatePower) < minPower) {
            rotatePower = Math.signum(rotatePower) * minPower;
        }

        return -(Math.max(-0.5, Math.min(0.5, rotatePower)));
    }

    /**
     * Strafes and drives smoothly to align perfectly in front of the AprilTag goal.
     */
    public void AutoAimManualMode(List<Double> aprilTagCoords, double targetDistance) {
        if (aprilTagCoords == null || aprilTagCoords.size() < 2) {
            this.drive(0, 0, 0);
            return;
        }

        double currentDistanceFB = aprilTagCoords.get(0);
        double currentDistanceLR = aprilTagCoords.get(1);

        double distanceErrorFB = currentDistanceFB - targetDistance;

        // FIX: Removed the negative sign. If the tag is to the right (positive),
        // we want positive strafe power to slide the robot right toward it.
        double distanceErrorLR = currentDistanceLR;

        // Absolute breakout deadzone to prevent jittering when completely settled
        if (Math.abs(distanceErrorLR) <= LR_DEADZONE_METERS && Math.abs(distanceErrorFB) <= FB_DEADZONE_METERS) {
            this.drive(0, 0, 0);
            return;
        }

        // 4. Calculate Forward/Backward Power with a smooth minimum ramp
        double FB_Power = 0.0;
        if (Math.abs(distanceErrorFB) > FB_DEADZONE_METERS) {
            FB_Power = distanceErrorFB * kP_Drive;
            FB_Power += Math.signum(FB_Power) * minPower;
        }

        // 5. Calculate Strafe Power with a smooth minimum ramp
        double LR_Power = 0.0;
        if (Math.abs(distanceErrorLR) > LR_DEADZONE_METERS) {
            LR_Power = distanceErrorLR * kP_Strafe;
            LR_Power += Math.signum(LR_Power) * minPower;
        }

        // 6. Heading is locked at 0.0 because you want pure translation without spinning
        double rotatePower = 0.0;

        // 7. Cap limits cleanly to prevent fast overshooting
        FB_Power = Math.max(-0.4, Math.min(0.4, FB_Power));
        LR_Power = Math.max(-0.4, Math.min(0.4, LR_Power));

        // 8. CRITICAL FIX: Plug LR_Power into the strafe parameter (the second slot)
        // and lock rotation at 0.0 so the heading never changes.
        this.drive(FB_Power, LR_Power, rotatePower);
    }
}