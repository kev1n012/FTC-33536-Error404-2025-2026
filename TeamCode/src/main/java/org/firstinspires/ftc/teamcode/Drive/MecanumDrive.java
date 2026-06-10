package org.firstinspires.ftc.teamcode.Drive;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import java.util.List;

public class MecanumDrive {

    //Make an empty class for the hardware
    private Hardware hw;

    private final double kP = 1.2;
    private final double minPower = 0.01;
    private final double DEADZONE_METERS = 0.02;

    public void init(Hardware hardware){
        //Fill the empty "hw" with the actual hardware
        this.hw = hardware;
    }
    public void drive(double forward, double strafe, double rotate){
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe  + rotate;
        double  backRightPower = forward + strafe - rotate;

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

    public void driveFieldRelative(double forward, double strafe, double rotate){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

    /**
     * Automatically rotates the robot to center on the AprilTag.
     * @param aprilTagCoords The coordinates list from your Limelight method
     */
    public double autoTarget(List<Double> aprilTagCoords) {
        if (aprilTagCoords == null || aprilTagCoords.size() < 2) {
            return 0.0;
        }

        //Get the rotation
        double distanceLR = aprilTagCoords.get(1);

        //Check if we are in the target deadzone
        if (Math.abs(distanceLR) <= DEADZONE_METERS) {
            this.drive(0, 0, 0);
            return 0.0;
        }

        double rotatePower = distanceLR * kP;

        // Ensure enough power to overcome friction
        if (Math.abs(rotatePower) < minPower) {
            rotatePower = Math.signum(rotatePower) * minPower;
        }

        // Bind power to min and max
        rotatePower = Math.max(-0.6, Math.min(0.6, rotatePower));
        return rotatePower;
    }
}


