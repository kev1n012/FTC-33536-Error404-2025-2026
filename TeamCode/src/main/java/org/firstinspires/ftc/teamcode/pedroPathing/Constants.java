package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    //TODO set the mass/weight in KG for the robot
    //DO not forget zero power acceleration!!
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(0.0)
            .forwardZeroPowerAcceleration(0.0)
            .lateralZeroPowerAcceleration(0.0)
            //TODO manually tune these
            .translationalPIDFCoefficients(new PIDFCoefficients(0.0,0.0,0.0,0.0))
            //TODO manually tune the PIDF for the heading as well
            .headingPIDFCoefficients(new PIDFCoefficients(0.0,0.0,0.0,0.0))
            //TODO tune using line follower (I think check Brogan video)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0,0.0,0.0,0.0, 0.0))
            //TODO curve tuning
            .centripetalScaling(0.0);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    //TODO Set motor names + direction
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //TODO after running test change and re-upload code
            //DO NOT FORGET line 18+19
            .xVelocity(0.0)
            .yVelocity(0.0);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}


//TODO check out helpful
// PedroPathing docs: https://pedropathing.com/docs/pathing
// Brogan M. Pratt on Youtube: https://www.youtube.com/watch?v=vihb2LPtSK0&list=PL66pc01PvvObKj6T6iCkZK9F_AQw4vyBH