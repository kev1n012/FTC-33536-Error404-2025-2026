//package org.firstinspires.ftc.teamcode.Vision;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.Hardware.Hardware;
//
//import java.util.Arrays;
//import java.util.List;
//
//public class LimeLightVision {
//
//    private Hardware hw;
//
//    LLResult llResult;
//    YawPitchRollAngles orientation;
//
//    List<Double> AprilTagCoords;
//
//    public void init(Hardware hardware){
//        this.hw = hardware;
//    }
//
//    public void StartVision(){
//        hw.limelight.start();
//    }
//
//    public void StopVision(){
//        hw.limelight.stop();
//    }
//
//    public void ChangePipeline(int Pipeline){
//        hw.limelight.pipelineSwitch(Pipeline);
//    }
//
//    public Pose3D UpdateBotPos() {
//        if (llResult != null && llResult.isValid()) {
//            return llResult.getBotpose_MT2();
//        }
//        return null;
//    }
//
//    public LLResult UpdateCamera(double headingDegrees) {
//        // Pass the double straight into the Limelight update function
//        hw.limelight.updateRobotOrientation(headingDegrees);
//        llResult = hw.limelight.getLatestResult();
//
//        if (llResult != null && llResult.isValid()) {
//            return llResult;
//        }
//        return null;
//    }
//
//    /**
//     * Calculates and returns the relative coordinate data for the primary visible AprilTag.
//     *
//     * @return A List of Doubles containing the following data in order:
//     * <ul>
//     * <li>Index 0: <b>distanceFB</b> - Distance forward and backward from the goal.</li>
//     * <li>Index 1: <b>distanceLR</b> - Distance to the right/left from the center of the goal.</li>
//     * <li>Index 2: <b>distanceH</b> - The vertical height/elevation of the AprilTag.</li>
//     * <li>Index 3: <b>distanceDD</b> - The true 3D diagonal distance to the tag.</li>
//     * </ul>
//     */
//    public List<Double> GetAprilTagCoords() {
//        if (llResult == null || !llResult.isValid()) {
//            return null;
//        }
//
//        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
//
//        if (fiducialResults == null || fiducialResults.isEmpty()) {
//            return null;
//        }
//
//        LLResultTypes.FiducialResult primaryTarget = fiducialResults.get(0);
//        Pose3D tagPoseCameraSpace = primaryTarget.getTargetPoseCameraSpace();
//
//        // Target offset tracking correction (-0.02M)
//        double distanceFB = tagPoseCameraSpace.getPosition().z - 0.02;
//        double distanceLR = tagPoseCameraSpace.getPosition().x;
//        double distanceH = tagPoseCameraSpace.getPosition().y;
//        double distanceDD = Math.sqrt(Math.pow(distanceFB, 2) + Math.pow(distanceLR, 2) + Math.pow(distanceH, 2));
//
//        AprilTagCoords = Arrays.asList(distanceFB, distanceLR, distanceH, distanceDD);
//        return AprilTagCoords;
//    }
//
//    /**
//     * Calculates continuous Flywheel Velocity based on a 3rd-degree cubic polynomial.
//     * Maps perfectly to presets: 1.5m->1450, 2.0m->1500, 2.5m->1600, 3.0m->1650
//     */
//    public double getVelocityCubicMath(double distance) {
//        // Safe boundary clipping matching your current lookup scale limits
//        if (distance < 1.50) distance = 1.50;
//        if (distance > 3.00) distance = 3.00;
//
//        return (-133.3333 * Math.pow(distance, 3)) + (900.0 * Math.pow(distance, 2)) - (1816.6667 * distance) + 2600.0;
//    }
//
//    /**
//     * Calculates continuous Trigger Open Threshold based on a 3rd-degree cubic polynomial.
//     * Maps perfectly to presets: 1.5m->1350, 2.0m->1450, 2.5m->1550, 3.0m->1600
//     */
//    public double getServoOpenCubicMath(double distance) {
//        if (distance < 1.50) distance = 1.50;
//        if (distance > 3.00) distance = 3.00;
//
//        return (-66.6667 * Math.pow(distance, 3)) + (400.0 * Math.pow(distance, 2)) - (583.3333 * distance) + 1550.0;
//    }
//
//    /**
//     * Calculates continuous Trigger Close Threshold based on a 3rd-degree cubic polynomial.
//     * Maps perfectly to presets: 1.5m->1100, 2.0m->1200, 2.5m->1300, 3.0m->1450
//     */
//    public double getServoCloseCubicMath(double distance) {
//        if (distance < 1.50) distance = 1.50;
//        if (distance > 3.00) distance = 3.00;
//
//        return (66.6667 * Math.pow(distance, 3)) - (400.0 * Math.pow(distance, 2)) + (983.3333 * distance) + 300.0;
//    }
//}





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