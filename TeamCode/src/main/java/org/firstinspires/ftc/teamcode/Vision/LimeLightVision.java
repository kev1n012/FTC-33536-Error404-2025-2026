package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import java.util.Arrays;
import java.util.List;

public class LimeLightVision {

    private Hardware hw;
    LLResult llResult;
    YawPitchRollAngles orientation;
    List<Double> AprilTagCoords;

    public static class ShotPreset {
        public final double distance;
        public final double velocity;
        public final double servoOpen;
        public final double servoClose;

        public ShotPreset(double distance, double velocity, double servoOpen, double servoClose) {
            this.distance = distance;
            this.velocity = velocity;
            this.servoOpen = servoOpen;
            this.servoClose = servoClose;
        }
    }

    private static final ShotPreset[] LOOKUP_TABLE = {
            new ShotPreset(1.5, 1450.0, 1350.0, 1100.0),
            new ShotPreset(2.0, 1500.0, 1450.0, 1200.0),
            new ShotPreset(2.5, 1600.0, 1550.0, 1300.0),
            new ShotPreset(3.0, 1650.0, 1600.0, 1450.0),
            new ShotPreset(3.25,1750.0, 1600.0,1500.0),
            new ShotPreset(3.5, 1800.0, 1750.0, 1550.0),
            new ShotPreset(3.75, 1850.0, 1800.0, 1600.0)

    };

    public void init(Hardware hardware){
        this.hw = hardware;
    }

    public void StartVision(){ hw.limelight.start(); }
    public void StopVision(){ hw.limelight.stop(); }
    public void ChangePipeline(int Pipeline){ hw.limelight.pipelineSwitch(Pipeline); }

    public Pose3D UpdateBotPos() {
        if (llResult != null && llResult.isValid()) return llResult.getBotpose_MT2();
        return null;
    }
    
    public LLResult UpdateCamera() {
        double yawInDegrees = Math.toDegrees(hw.imu.getHeading());
        hw.limelight.updateRobotOrientation(yawInDegrees);

        llResult = hw.limelight.getLatestResult();
        return (llResult != null && llResult.isValid()) ? llResult : null;
    }

    public List<Double> GetAprilTagCoords() {
        if (llResult == null || !llResult.isValid()) return null;
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
        if (fiducialResults == null || fiducialResults.isEmpty()) return null;

        LLResultTypes.FiducialResult primaryTarget = fiducialResults.get(0);
        Pose3D tagPoseCameraSpace = primaryTarget.getTargetPoseCameraSpace();

        double distanceFB = tagPoseCameraSpace.getPosition().z - 0.02;
        double distanceLR = tagPoseCameraSpace.getPosition().x;
        double distanceH = tagPoseCameraSpace.getPosition().y;
        double distanceDD = Math.sqrt(Math.pow(distanceFB, 2) + Math.pow(distanceLR, 2) + Math.pow(distanceH, 2));

        AprilTagCoords = Arrays.asList(distanceFB, distanceLR, distanceH, distanceDD);
        return AprilTagCoords;
    }

    /**
     * Finds the bounding rows in the lookup table and calculates continuous linear
     * interpolations for Flywheel Velocity, Servo Open, and Servo Close outputs.
     */
    public ShotPreset getInterpolatedShot(double targetDistance) {
        // Enforce hard boundaries based on the edges of our table
        double distance = Math.max(LOOKUP_TABLE[0].distance,
                Math.min(LOOKUP_TABLE[LOOKUP_TABLE.length - 1].distance, targetDistance));

        // Exact lower bound match or out-of-bounds low fallback
        if (distance <= LOOKUP_TABLE[0].distance) {
            return LOOKUP_TABLE[0];
        }
        // Exact upper bound match or out-of-bounds high fallback
        if (distance >= LOOKUP_TABLE[LOOKUP_TABLE.length - 1].distance) {
            return LOOKUP_TABLE[LOOKUP_TABLE.length - 1];
        }

        // Search the array to locate which two presets the robot is currently sitting between
        int upperIndex = 1;
        while (upperIndex < LOOKUP_TABLE.length && LOOKUP_TABLE[upperIndex].distance < distance) {
            upperIndex++;
        }

        ShotPreset low = LOOKUP_TABLE[upperIndex - 1];
        ShotPreset high = LOOKUP_TABLE[upperIndex];

        // Calculate the interpolation percentage (where are we between point A and point B)
        double pct = (distance - low.distance) / (high.distance - low.distance);

        // Blend the target values proportionally across the distance interval
        double interpolatedVel   = low.velocity + pct * (high.velocity - low.velocity);
        double interpolatedOpen  = low.servoOpen + pct * (high.servoOpen - low.servoOpen);
        double interpolatedClose = low.servoClose + pct * (high.servoClose - low.servoClose);

        return new ShotPreset(distance, interpolatedVel, interpolatedOpen, interpolatedClose);
    }
}