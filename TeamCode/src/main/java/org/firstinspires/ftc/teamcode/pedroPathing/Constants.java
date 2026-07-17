package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    //TODO set the mass/weight in KG for the robot
    //DO not forget zero power acceleration!!!
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.20)
            .headingPIDFCoefficients(new PIDFCoefficients(1.1, 0,0.05,0.4))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.2, 0.07188766305026026, 0.0017563625703268448))
            .centripetalScaling(0);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.MM)
            .forwardPodY(92)
            .strafePodX(90)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.97,  // Changed from 0.99
            100,
            1,
            1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(66.86832698311392)
            .yVelocity(58.25653460645301);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

/*
* Tuning Tips
    PedroPathing does not activate heading and translational PIDF correction for drive tuning. If you would like to test all three of them, navigate yourself to the Line Tuner in the Manual folder. Use it to adjust BrakingStrength, path constraints and making sure all PIDFs are working well together.
    Increasing your drive PIDF will make the robot move more quickly along the path, at the risk of more overshoot at the end of the path.
    Decreasing your drive PIDF will make the robot move more slowly and reduce the overshoot at the end of the path.
    Adjusting the BrakingStrength can significantly help manage how smoothly the robot decelerates as it reaches the end of its path.
    If the robot drives quickly during the middle of the path but abruptly slows down as it reaches the end of the path, this may be caused by the transition between the main and secondary PIDs. This problem may also be addressed through lowering the BrakingStrength.
*
* */


//TODO check out helpful
// PedroPathing docs: https://pedropathing.com/docs/pathing
// Brogan M. Pratt on Youtube: https://www.youtube.com/watch?v=vihb2LPtSK0&list=PL66pc01PvvObKj6T6iCkZK9F_AQw4vyBH