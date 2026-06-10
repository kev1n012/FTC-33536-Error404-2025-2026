package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import java.util.Arrays;
import java.util.List;

public class LimeLightVision {

    //Make an empty class for the hardware
    private Hardware hw;

    LLResult llResult;
    YawPitchRollAngles orientation;

    List<Double> AprilTagCoords;

    public void init(Hardware hardware){
        //Fill the empty "hw" with the actual hardware
        this.hw = hardware;
    }

    public void StartVision(){
        hw.limelight.start();
    }

    public void StopVision(){hw.limelight.stop();}

    public void ChangePipeline(int Pipeline){
        hw.limelight.pipelineSwitch(Pipeline);
    }

    public Pose3D UpdateBotPos() {
        if (llResult != null && llResult.isValid()) {
            //Return the bot position as Pose3D
            return llResult.getBotpose_MT2();
        }
        return null;
    }
    public LLResult UpdateCamera() {
        orientation = hw.imu.getRobotYawPitchRollAngles();
        hw.limelight.updateRobotOrientation(orientation.getYaw());
        llResult = hw.limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            return llResult;
        }
        return null;
    }


    /**
     * Calculates and returns the relative coordinate data for the primary visible AprilTag.
     *
     * @return A List of Doubles containing the following data in order:
     * <ul>
     * <li>Index 0: <b>distanceFB</b> - Distance forward and backward from the goal.</li>
     * <li>Index 1: <b>distanceLR</b> - Distance to the right/left from the center of the goal.</li>
     * <li>Index 2: <b>distanceH</b> - The vertical height/elevation of the AprilTag.</li>
     * <li>Index 3: <b>distanceDD</b> - The true 3D diagonal distance to the tag (useful for turrets).</li>
     * </ul>
     */
    public List<Double> GetAprilTagCoords() {

        if (llResult == null || !llResult.isValid()) {
            return null;
        }
        // Get the list of visible AprilTags
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();

        if (fiducialResults == null || fiducialResults.isEmpty()) {
            return null;
        }

        // Get the first primary target
        LLResultTypes.FiducialResult primaryTarget = fiducialResults.get(0);

        // Get its pose relative to the robot space
        Pose3D tagPoseCameraSpace = primaryTarget.getTargetPoseCameraSpace();
        ///NOTE Made it so its -0.02M as an correction offset
        double distanceFB = tagPoseCameraSpace.getPosition().z - 0.02; // distance Forward and backward
        double distanceLR = tagPoseCameraSpace.getPosition().x; // distance Left and Right from the centre of the tag
        double distanceH = tagPoseCameraSpace.getPosition().y; // distance Height to the april tag
        double distanceDD = Math.sqrt(Math.pow(distanceFB,2) + Math.pow(distanceLR, 2) + Math.pow(distanceH, 2)); // distance Diagonal Distance to the tag (Used for turrets)

        // Fill and return the list
        AprilTagCoords = Arrays.asList(distanceFB, distanceLR, distanceH, distanceDD);
        return AprilTagCoords;
    }
}
