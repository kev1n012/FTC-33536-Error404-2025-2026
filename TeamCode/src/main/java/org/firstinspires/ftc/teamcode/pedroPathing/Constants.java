package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
            .mass(5.0)
            .forwardZeroPowerAcceleration(-39.761636841755085)
            .lateralZeroPowerAcceleration( -48.73472159782229)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04,0,0.01,0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.4, 0,0.003,0.035));

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(92/2.54)
            .strafePodX(90/2.54)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    //TODO Set motor names + direction
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


//TODO check out helpful
// PedroPathing docs: https://pedropathing.com/docs/pathing
// Brogan M. Pratt on Youtube: https://www.youtube.com/watch?v=vihb2LPtSK0&list=PL66pc01PvvObKj6T6iCkZK9F_AQw4vyBH