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
public class RED_SHORT_PL_G_L1_L2_L3 extends OpMode{

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
    private final Pose line1GateCurve = new Pose((141.5 - 58.5),80);
    private final Pose line2Start = new Pose((141.5 - 55.5),85.5, Math.toRadians(90-(180-90)));
    private final Pose line2Curve = new Pose((141.5 - 58.5),56);
    private final Pose line2End = new Pose((141.5 - 12),62, Math.toRadians(90-(180-90)));
    private final Pose line2returnCurve = new Pose((141.5 - 47.5),67);
    private final Pose line2GateCurve = new Pose((141.5 - 60), 59);
    private final Pose GatePose = new Pose((141.5 - 17.5),75, Math.toRadians(90-(180-90)));
    private final Pose line3Start = new Pose((141.5 - 55.5),85.5, Math.toRadians((90-(180-90))));
    private final Pose line3Curve = new Pose((141.5 - 70),30);
    private final Pose line3End = new Pose((141.5 - 11), 38, Math.toRadians((90-(180-90))));
    private final Pose offPose = new Pose((141.5 - 26),90, Math.toRadians(90-(270-90)));








    private PathChain driveStartPosShootPos;
    private PathChain driveStartL1EndL1;
    private PathChain driveEndL1GatePose;
    private PathChain driveEndL1ShootPose;
    private PathChain driveStartL2EndL2;
    private PathChain driveEndL2ShootPose;
    private PathChain driveEndL2GatePose;
    private PathChain driveGatePoseShootPose;
    private PathChain driveStartL3EndL3;
    private PathChain driveEndL3ShootPose;
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
        driveEndL1GatePose = follower.pathBuilder()
                .addPath(new BezierCurve(line1End, line1GateCurve, GatePose))
                .setLinearHeadingInterpolation(line1End.getHeading(), GatePose.getHeading())
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
        driveEndL2GatePose = follower.pathBuilder()
                .addPath(new BezierCurve(line2End, line2GateCurve, GatePose))
                .setLinearHeadingInterpolation(line2End.getHeading(),GatePose.getHeading())
                .build();
        driveGatePoseShootPose = follower.pathBuilder()
                .addPath(new BezierLine(GatePose, shootPose))
                .setLinearHeadingInterpolation(GatePose.getHeading(), shootPose.getHeading())
                .build();
        driveStartL3EndL3 = follower.pathBuilder()
                .addPath(new BezierCurve(line3Start, line3Curve, line3End))
                .setLinearHeadingInterpolation(line3Start.getHeading(), line3End.getHeading())
                .build();
        driveEndL3ShootPose = follower.pathBuilder()
                .addPath(new BezierLine(line3End, shootPose))
                .setLinearHeadingInterpolation(line3End.getHeading(), shootPose.getHeading())
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
                    if (pathTimer.getElapsedTimeSeconds() < 4.4) {
                        SHORT = true;
                        intake.setPower(-0.75);
                        helper_motor.setPower(-0.85);
                        flywheelL.setVelocity(SHOOTER_POWER_SHORT);
                        flywheelR.setVelocity(SHOOTER_POWER_SHORT);
                        if (SHORT && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_SHORT) {
                            shooter_servo.setPosition(OPEN_SERVO_POS);
                        } else if (SHORT && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_SHORT) {
                            shooter_servo.setPosition(CLOSE_SERVO_POS);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4.4) {
                        intake.setPower(0);
                        helper_motor.setPower(0);
                        flywheelR.setVelocity(ShooterVel());
                        flywheelL.setVelocity(ShooterVel());
                        shooter_servo.setPosition(CLOSE_SERVO_POS);
                        setPathState(RED_SHORT_PL_L1.PathState.DRIVE_TAKE_LINE_1);
                    }
                }
                break;


            case DRIVE_TAKE_LINE_1:
                if (!follower.isBusy() ) {
                    telemetry.addLine("Taking Line 1");
                    intake.setPower(-0.7);
                    helper_motor.setPower(-0.8);
                    follower.followPath(driveStartL1EndL1);
                    setPathState(RED_SHORT_PL_L1.PathState.DRIVE_END_LINE_1_GATE_POS);
                }
                break;


            case DRIVE_END_LINE_1_GATE_POS:
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2)|| pathTimer.getElapsedTimeSeconds() > 3) {

                    helper_motor.setPower(0);
                    flywheelR.setVelocity(ShooterVel());
                    flywheelL.setVelocity(ShooterVel());
                    telemetry.addLine("Shooting Line 1");
                    follower.followPath(driveEndL1GatePose);
                    setPathState(RED_SHORT_PL_L1.PathState.DRIVE_GATE_POS_SHOOT_POS);
                }
                break;

            case DRIVE_GATE_POS_SHOOT_POS:
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1)|| pathTimer.getElapsedTimeSeconds() > 3) {
                    telemetry.addLine("Gate opened, shooting Line 2");
                    flywheelR.setVelocity(ShooterVel());
                    flywheelL.setVelocity(ShooterVel());
                    follower.followPath(driveGatePoseShootPose);
                    setPathState(RED_SHORT_PL_L1.PathState.SHOOT_LINE1);
                }
                break;

            case SHOOT_LINE1:
                if (!follower.isBusy()) {
                    telemetry.addLine("Shot Line 1");
                    if (pathTimer.getElapsedTimeSeconds() < 4.4) {
                        SHORT = true;
                        intake.setPower(-0.75);
                        helper_motor.setPower(-0.85);
                        flywheelL.setVelocity(SHOOTER_POWER_SHORT);
                        flywheelR.setVelocity(SHOOTER_POWER_SHORT);
                        if (SHORT && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_SHORT) {
                            shooter_servo.setPosition(OPEN_SERVO_POS);
                        } else if (SHORT && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_SHORT) {
                            shooter_servo.setPosition(CLOSE_SERVO_POS);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4.4) {

                        setPathState(RED_SHORT_PL_L1.PathState.DRIVE_START_LINE_2_END_LINE_2);
                        intake.setPower(0);
                        helper_motor.setPower(0);
                        flywheelR.setVelocity(ShooterVel());
                        flywheelL.setVelocity(ShooterVel());
                        shooter_servo.setPosition(CLOSE_SERVO_POS);
                    }


                }
                break;


            case DRIVE_START_LINE_2_END_LINE_2:
                if (!follower.isBusy()) {
                    telemetry.addLine("Take Line 2");
                    intake.setPower(-0.7);
                    helper_motor.setPower(-0.8);
                    follower.followPath(driveStartL2EndL2, true);
                    setPathState(RED_SHORT_PL_L1.PathState.DRIVE_END_LINE_2_SHOOT_POS);
                }
                break;

            case DRIVE_END_LINE_2_SHOOT_POS:
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2)|| pathTimer.getElapsedTimeSeconds() > 3) {
                    telemetry.addLine("Shooting Line 2");
                    flywheelR.setVelocity(ShooterVel());
                    flywheelL.setVelocity(ShooterVel());
                    helper_motor.setPower(0);
                    follower.followPath(driveEndL2ShootPose, true);
                    setPathState(RED_SHORT_PL_L1.PathState.SHOOT_LINE_2);
                }
                break;


            case SHOOT_LINE_2:
                if (!follower.isBusy()) {
                    telemetry.addLine("Shot Line 2");
                    if (pathTimer.getElapsedTimeSeconds() < 5) {
                        SHORT = true;
                        intake.setPower(-0.75);
                        helper_motor.setPower(-0.85);
                        flywheelL.setVelocity(SHOOTER_POWER_SHORT);
                        flywheelR.setVelocity(SHOOTER_POWER_SHORT);
                        if (SHORT && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_SHORT) {
                            shooter_servo.setPosition(OPEN_SERVO_POS);
                        } else if (SHORT && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_SHORT) {
                            shooter_servo.setPosition(CLOSE_SERVO_POS);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5) {

                        setPathState(RED_SHORT_PL_L1.PathState.DRIVE_START_LINE_3_END_LINE_3);
                        intake.setPower(0);
                        helper_motor.setPower(0);
                        flywheelR.setVelocity(ShooterVel());
                        flywheelL.setVelocity(ShooterVel());
                        shooter_servo.setPosition(CLOSE_SERVO_POS);
                    }

                }

                break;


            case DRIVE_START_LINE_3_END_LINE_3:
                if (!follower.isBusy()) {
                    telemetry.addLine("Take Line 3");
                    intake.setPower(-0.7);
                    helper_motor.setPower(-0.8);
                    follower.followPath(driveStartL3EndL3);
                    setPathState(RED_SHORT_PL_L1.PathState.DRIVE_END_LINE_3_SHOOT_POS);
                }
                break;


            case DRIVE_END_LINE_3_SHOOT_POS:
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) || pathTimer.getElapsedTimeSeconds() > 4) {
                    telemetry.addLine("Shooting Line 3");

                    helper_motor.setPower(0);
                    flywheelR.setVelocity(ShooterVel());
                    flywheelL.setVelocity(ShooterVel());
                    follower.followPath(driveEndL3ShootPose);
                    setPathState(RED_SHORT_PL_L1.PathState.SHOOT_LINE_3);
                }
                break;


            case SHOOT_LINE_3:
                if (!follower.isBusy()) {
                    telemetry.addLine("Shot Line 3");
                    if (pathTimer.getElapsedTimeSeconds() < 5.5) {
                        SHORT = true;
                        intake.setPower(-0.75);
                        helper_motor.setPower(-0.85);
                        flywheelL.setVelocity(SHOOTER_POWER_SHORT);
                        flywheelR.setVelocity(SHOOTER_POWER_SHORT);
                        if (SHORT && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_SHORT) {
                            shooter_servo.setPosition(OPEN_SERVO_POS);
                        } else if (SHORT && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_SHORT) {
                            shooter_servo.setPosition(CLOSE_SERVO_POS);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5.5) {
                        follower.followPath(driveshootPosOffPos, true);
                        setPathState(RED_SHORT_PL_L1.PathState.MOVE_OFF_LINE);
                        intake.setPower(0);
                        helper_motor.setPower(0);
                        flywheelR.setVelocity(0);
                        flywheelL.setVelocity(0);
                        shooter_servo.setPosition(CLOSE_SERVO_POS);
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
