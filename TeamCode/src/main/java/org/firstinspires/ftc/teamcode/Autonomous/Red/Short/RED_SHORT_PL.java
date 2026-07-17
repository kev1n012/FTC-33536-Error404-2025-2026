package org.firstinspires.ftc.teamcode.Autonomous.Red.Short;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class RED_SHORT_PL extends OpMode  {


    private DcMotorEx intake;
    private DcMotorEx helper_motor;
    private DcMotorEx flywheelR, flywheelL;
    private Servo shooter_servo;

    private double rightFlywheelSpeed = 0;
    private double leftFlywheelSpeed = 0;

    private static final int TICKS_PER_REVOLUTION = 28;

    public void setFlywheels(double speed) {
        this.flywheelL.setVelocity(speed);
        this.flywheelR.setVelocity(speed);
    }

    private void initializeHardware() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        helper_motor = hardwareMap.get(DcMotorEx.class, "helper_motor");
        flywheelR = hardwareMap.get(DcMotorEx.class, "flywheelR");
        flywheelL = hardwareMap.get(DcMotorEx.class, "flywheelL");
        shooter_servo = hardwareMap.get(Servo.class, "shooter_servo");

    }


    private void configureMotors() {
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        helper_motor.setDirection(DcMotor.Direction.REVERSE);
        helper_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelR.setDirection(DcMotor.Direction.FORWARD);
        flywheelL.setDirection(DcMotor.Direction.REVERSE);
        setMotorRunMode(flywheelR);
        setMotorRunMode(flywheelL);
    }

    private void setMotorRunMode(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double calculateRPM(DcMotorEx motor, double ticksPerRevolution) {
        return (motor.getVelocity() / ticksPerRevolution) * 60.0;
    }

    private double ShooterVel() {
        Pose curPos = follower.getPose();

        double dx = SHOOT_POS_X - curPos.getX();
        double dy = SHOOT_POS_Y - curPos.getY();
        double distanceFromSpot = Math.sqrt(dx * dx + dy * dy);

        double pctFromSpot = 1.0 - (distanceFromSpot / MAX_DROP_DISTANCE);

        pctFromSpot = Math.max(0.0, Math.min(1.0, pctFromSpot));

        return IDLE_VEL + (pctFromSpot * (MAX_SHOOT_VEL - IDLE_VEL));
    }

    private static final double SHOOT_POS_X = 58.0;
    private static final double SHOOT_POS_Y = 82.0;

    private final double MAX_SHOOT_VEL = 1450.0;
    private final double IDLE_VEL = 500.0;
    private final double MAX_DROP_DISTANCE = 24.0;

    private static final double OPEN_SERVO_POS = 0.25;
    private static final double CLOSE_SERVO_POS = 0;
    private  int DESIRED_FLYWHEEL_SPEED_LONG = 3900;
    private  int OPEN_SHOOTER_SERVO_LONG = 1575;
    private int CLOSE_SHOOTER_SERVO_LONG = 1450;
    private static final double SHOOTER_POWER_LONG = 1700;
    private int DESIRED_FLYWHEEL_SPEED_SHORT = 2500;
    private int OPEN_SHOOTER_SERVO_SHORT = 1350;
    private int CLOSE_SHOOTER_SERVO_SHORT = 1100;
    private static final double SHOOTER_POWER_SHORT = 1450;
    private boolean SHORT;
    private boolean LONG;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    RED_SHORT_PL_L1.PathState pathState;

    private final Pose startPose = new Pose((141.5 - 19),121, Math.toRadians(90-(144-90)));
    private final Pose shootPose = new Pose((141.5 - 55.5),85.5, Math.toRadians(90-(138-90)));
    private final Pose line1Start = new Pose((141.5 - 55.5),85.5, Math.toRadians(90-(180-90)));
    private final Pose line1Curve = new Pose((141.5 - 40.5), 84.5, Math.toRadians(90-(180-90)));
    private final Pose line1End = new Pose((141.5 - 20), 84.5, Math.toRadians(90-(180-90)));
    private final Pose line2Start = new Pose((141.5 - 55.5),85.5, Math.toRadians(90-(180-90)));
    private final Pose line2Curve = new Pose((141.5 - 58.5),58);
    private final Pose line2End = new Pose((141.5 - 12.5),62, Math.toRadians(90-(180-90)));
    private final Pose line2returnCurve = new Pose((141.5 - 47.5),67);
    private final Pose offPose = new Pose((141.5 - 14),115, Math.toRadians(90-(270-90)));



    private PathChain driveStartPosShootPos;
    private PathChain driveStartL1EndL1;
    private PathChain driveEndL1ShootPose;
    private PathChain driveStartL2EndL2;
    private PathChain driveEndL2ShootPose;
    private PathChain driveshootPosOffPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveStartL1EndL1 = follower.pathBuilder()
                .addPath(new BezierCurve(line1Start, line1Curve, line1End))
                .setLinearHeadingInterpolation(line1Start.getHeading(), line1End.getHeading())
                .build();
        driveEndL1ShootPose = follower.pathBuilder()
                .addPath(new BezierLine(line1End,shootPose))
                .setLinearHeadingInterpolation(line1End.getHeading(), shootPose.getHeading())
                .build();
        driveStartL2EndL2 = follower.pathBuilder()
                .addPath(new BezierCurve(line2Start, line2Curve, line2End))
                .setLinearHeadingInterpolation(line2Start.getHeading(), line2End.getHeading())
                .build();
        driveEndL2ShootPose = follower.pathBuilder()
                .addPath(new BezierCurve(line2End, line2returnCurve, shootPose))
                .setLinearHeadingInterpolation(line2End.getHeading(), shootPose.getHeading())
                .build();
        driveshootPosOffPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,offPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), offPose.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_START_POS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(RED_SHORT_PL_L1.PathState.SHOOT_PRELOAD);
                flywheelR.setVelocity(ShooterVel());
                flywheelL.setVelocity(ShooterVel());
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    telemetry.addLine("Done Path 1");
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        SHORT = true;
                        intake.setPower(-0.75);
                        helper_motor.setPower(-0.8);
                        flywheelL.setVelocity(SHOOTER_POWER_SHORT);
                        flywheelR.setVelocity(SHOOTER_POWER_SHORT);
                        if (SHORT && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_SHORT) {
                            shooter_servo.setPosition(OPEN_SERVO_POS);
                        } else if (SHORT && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_SHORT) {
                            shooter_servo.setPosition(CLOSE_SERVO_POS);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 8) {
                        intake.setPower(0);
                        helper_motor.setPower(0);
                        flywheelR.setVelocity(0);
                        flywheelL.setVelocity(0);
                        shooter_servo.setPosition(CLOSE_SERVO_POS);
                        follower.followPath(driveshootPosOffPos);
                        setPathState(RED_SHORT_PL_L1.PathState.MOVE_OFF_LINE);
                    }
                }
                break;

            case MOVE_OFF_LINE:
                if (!follower.isBusy()){
                    telemetry.addLine("Done all Paths");
                }
                break;
            default:
                telemetry.addLine("No State Command");
                break;

        }
    }


    public void setPathState(RED_SHORT_PL_L1.PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = RED_SHORT_PL_L1.PathState.DRIVE_START_POS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        configureMotors();
        shooter_servo.setPosition(0);
        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        rightFlywheelSpeed = flywheelR.getVelocity();
        leftFlywheelSpeed =  flywheelL.getVelocity();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());



    }
}


