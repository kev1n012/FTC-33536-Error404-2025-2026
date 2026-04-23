package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class ColorSensorVision {
    private Hardware hw;

    NormalizedRGBA colors;

    public enum DetectableColors{
        PURPLE,
        GREEN,
        UNKNOWN
    }
    public void init(Hardware hardware){
        this.hw = hardware;
    }

    public DetectableColors getDetectedColor(NormalizedColorSensor SelectedSensor, Telemetry telemetry){
        // Returns: R = red / G = green / B = blue / A = alpha (brightness)
        colors =  SelectedSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        //TODO turn this into a summary once colors have been set
        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        //TODO add tune the color sensors then add back the 'if' statements with the respected values
        /*
        if(normRed > 1.0 && normBlue > 1.0 && normGreen < 0.5){
            return DetectableColors.PURPLE;
        } else if(normRed < 0.5 && normBlue < 0.5 && normGreen > 1.0){
            return  DetectableColors.GREEN;
        } else{
            return DetectableColors.UNKNOWN;
        }
         */

        //TODO Remove this once the values have been set
        return DetectableColors.UNKNOWN;
    }
}
