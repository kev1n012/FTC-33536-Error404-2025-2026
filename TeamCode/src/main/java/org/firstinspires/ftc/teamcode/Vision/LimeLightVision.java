package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class LimeLightVision {

    //Make an empty class for the hardware
    private Hardware hw;

    LLResult llResult;
    YawPitchRollAngles orientation;

    public void init(Hardware hardware){
        //Fill the empty "hw" with the actual hardware
        this.hw = hardware;
    }

    public void StartVision(){
        hw.limelight.start();
    }

    public void ChangePipeline(int Pipeline){
        hw.limelight.pipelineSwitch(Pipeline);
    }

    public Pose3D updateVision(){
        orientation = hw.imu.getRobotYawPitchRollAngles();
        hw.limelight.updateRobotOrientation(orientation.getYaw());
        llResult = hw.limelight.getLatestResult();

        if (llResult != null && llResult.isValid()){
            //Return the bot position as Pose3D
            return llResult.getBotpose_MT2();
        } else{
            return null;
        }

    }
}
