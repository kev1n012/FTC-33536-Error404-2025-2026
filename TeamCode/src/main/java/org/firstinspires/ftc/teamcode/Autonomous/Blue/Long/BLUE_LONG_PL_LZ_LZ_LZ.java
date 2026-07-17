package org.firstinspires.ftc.teamcode.Autonomous.Blue.Long;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BLUE_LONG_PL_LZ_LZ_LZ extends OpMode {
    private double rightFlywheelSpeed = 0;
    private double leftFlywheelSpeed = 0;
    
    private Hardware hw;
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
    private  int OPEN_SHOOTER_SERVO_LONG = 1625;
    private int CLOSE_SHOOTER_SERVO_LONG = 1625;
    private static final double SHOOTER_POWER_LONG = 1700;
    private int DESIRED_FLYWHEEL_SPEED_SHORT = 2500;
    private int OPEN_SHOOTER_SERVO_SHORT = 1350;
    private int CLOSE_SHOOTER_SERVO_SHORT = 1100;
    private static final double SHOOTER_POWER_SHORT = 1450;
    private boolean SHORT;
    private boolean LONG;
    private Follower follower;
    private Timer pathTimer, opModeTimer;





    BLUE_LONG_PL_LZ_L3_LZ.PathStateLong pathState;

    private final Pose startPoseLong = new Pose(55.5,8, Math.toRadians(90));
    private final Pose shootPoseLong = new Pose(55.5,15, Math.toRadians(113));
    private final Pose LoadingZoneUpPose = new Pose(10,23.5,Math.toRadians(270));
    private final Pose LoadingZoneUpCurve = new Pose(12,33);
    private final Pose LoadingZoneSidePose = new Pose(16,18, Math.toRadians(180));
    private final Pose LoadingZoneSideCurve = new Pose(52, 18);
    private final Pose line3LongStart = new Pose(55.5,15, Math.toRadians(115));
    private final Pose line3LongCurve = new Pose(60, 36);
    private final Pose line3LongEnd = new Pose(16, 41, Math.toRadians(180));
    private final Pose offPoseLong = new Pose(59,36, Math.toRadians(90));



    private PathChain driveStartLongPosShootLongPos;
    private PathChain driveShootLongPosStartL3;
    private PathChain driveStartLongL3EndL3;
    private PathChain driveEndL3ShootLongPose;
    private PathChain driveShootLongPosLoadPosUpCurve;
    private PathChain driveLoadPosUpShootLongPos;
    private PathChain driveShootLongPosLoadPosSideCurve;
    private PathChain driveLoadPosSideShootLongPos;
    private PathChain driveShootLongPosOffLongPos;

    public void buildPaths() {
        driveStartLongPosShootLongPos = follower.pathBuilder()
                .addPath(new BezierLine(startPoseLong,shootPoseLong))
                .setLinearHeadingInterpolation(startPoseLong.getHeading(), shootPoseLong.getHeading())
                .build();
        driveShootLongPosLoadPosUpCurve = follower.pathBuilder()
                .addPath(new BezierCurve(shootPoseLong, LoadingZoneUpCurve,LoadingZoneUpPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), LoadingZoneUpPose.getHeading())
                .build();
        driveLoadPosUpShootLongPos = follower.pathBuilder()
                .addPath(new BezierCurve(LoadingZoneUpPose, LoadingZoneUpCurve, shootPoseLong))
                .setLinearHeadingInterpolation(LoadingZoneUpPose.getHeading(), shootPoseLong.getHeading())
                .build();
        driveShootLongPosStartL3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseLong,line3LongStart))
                .setLinearHeadingInterpolation(shootPoseLong.getHeading(), line3LongStart.getHeading())
                .build();
        driveStartLongL3EndL3 = follower.pathBuilder()
                .addPath(new BezierCurve(line3LongStart, line3LongCurve, line3LongEnd))
                .setLinearHeadingInterpolation(line3LongStart.getHeading(), line3LongEnd.getHeading())
                .build();
        driveEndL3ShootLongPose = follower.pathBuilder()
                .addPath(new BezierLine(line3LongEnd,shootPoseLong))
                .setLinearHeadingInterpolation(line3LongEnd.getHeading(), shootPoseLong.getHeading())
                .build();
        driveShootLongPosLoadPosSideCurve = follower.pathBuilder()
                .addPath(new BezierCurve(shootPoseLong, LoadingZoneSideCurve, LoadingZoneSidePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), LoadingZoneSidePose.getHeading())
                .build();
        driveLoadPosSideShootLongPos = follower.pathBuilder()
                .addPath(new BezierLine(LoadingZoneSidePose, shootPoseLong))
                .setLinearHeadingInterpolation(LoadingZoneSidePose.getHeading(), shootPoseLong.getHeading())
                .build();
        driveShootLongPosOffLongPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseLong,offPoseLong))
                .setLinearHeadingInterpolation(shootPoseLong.getHeading(), offPoseLong.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_START_LONG_POS_SHOOT_LONG_POS:
                follower.followPath(driveStartLongPosShootLongPos, true);
                setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.SHOOT_PRELOAD_LONG);
                hw.setFlywheels(ShooterVel());
                
                break;
            case SHOOT_PRELOAD_LONG:
                if (!follower.isBusy() ) {
                    telemetry.addLine("Done Path 1");
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        LONG = true;
                        hw.intake.setPower(-0.8);
                        hw.helperMotor.setPower(-0.8);
                        hw.setFlywheels(SHOOTER_POWER_LONG);
                        if (LONG && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_LONG) {
                            hw.shooterServo.setPosition(OPEN_SERVO_POS);
                        } else if (LONG && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_LONG) {
                            hw.shooterServo.setPosition(CLOSE_SERVO_POS);
                        }

                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5.5) {
                        setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.DRIVE_SHOOT_POS_LOAD_ZONE_POS);
                        hw.intake.setPower(0);
                        hw.helperMotor.setPower(0);
                        hw.setFlywheels(ShooterVel());
                        hw.shooterServo.setPosition(CLOSE_SERVO_POS);

                    }
                }
                break;
            case DRIVE_SHOOT_POS_LOAD_ZONE_POS:
                if (!follower.isBusy()) {
                    telemetry.addLine("Taking LoadZone");
                    hw.intake.setPower(-0.8);
                    hw.helperMotor.setPower(-0.8);
                    follower.followPath(driveShootLongPosLoadPosUpCurve);
                    setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.DRIVE_LOAD_ZONE_POS_SHOOT_POS);
                }
                break;
            case DRIVE_LOAD_ZONE_POS_SHOOT_POS:
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) || pathTimer.getElapsedTimeSeconds() > 3) {
                    telemetry.addLine("Took LoadZone");

                    hw.helperMotor.setPower(0);
                    follower.followPath(driveLoadPosUpShootLongPos);
                    setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.SHOOT_LZ_1);
                }
                break;
            case SHOOT_LZ_1:
                if (!follower.isBusy() ) {
                    telemetry.addLine("Shot LZ 1");
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        LONG = true;
                        hw.intake.setPower(-0.8);
                        hw.helperMotor.setPower(-0.85);
                        hw.setFlywheels(SHOOTER_POWER_LONG);
                        if (LONG && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_LONG) {
                            hw.shooterServo.setPosition(OPEN_SERVO_POS);
                        } else if (LONG && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_LONG) {
                            hw.shooterServo.setPosition(CLOSE_SERVO_POS);
                        }

                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5) {
                        setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.DRIVE_SHOOT_POS_LOAD_ZONE_SIDE_POS);
                        hw.intake.setPower(0);
                        hw.helperMotor.setPower(0);
                        hw.setFlywheels(ShooterVel());
                        hw.shooterServo.setPosition(CLOSE_SERVO_POS);

                    }
                }
                break;

            case DRIVE_SHOOT_POS_LOAD_ZONE_SIDE_POS:
                if (!follower.isBusy()) {
                    telemetry.addLine("Taking LoadZone");
                    hw.intake.setPower(-0.8);
                    hw.helperMotor.setPower(-0.8);
                    follower.followPath(driveShootLongPosLoadPosSideCurve);
                    setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.DRIVE_LOAD_ZONE_SIDE_POS_SHOOT_POS);
                }
                break;

            case DRIVE_LOAD_ZONE_SIDE_POS_SHOOT_POS:
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    telemetry.addLine("Took LoadZone");

                    hw.helperMotor.setPower(0);
                    follower.followPath(driveLoadPosSideShootLongPos);
                    setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.SHOOT_LZ_2);
                }
                break;

            case SHOOT_LZ_2:
                if (!follower.isBusy() ) {
                    telemetry.addLine("Shot LZ 2");
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        LONG = true;
                        hw.intake.setPower(-0.8);
                        hw.helperMotor.setPower(-0.85);
                        hw.setFlywheels(SHOOTER_POWER_LONG);
                        if (LONG && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_LONG) {
                            hw.shooterServo.setPosition(OPEN_SERVO_POS);
                        } else if (LONG && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_LONG) {
                            hw.shooterServo.setPosition(CLOSE_SERVO_POS);
                        }

                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5) {

                        setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.DRIVE_SHOOT_POS_LOAD_ZONE_SIDE_POS_2);
                        hw.intake.setPower(0);
                        hw.helperMotor.setPower(0);
                        hw.setFlywheels(ShooterVel());
                        hw.shooterServo.setPosition(CLOSE_SERVO_POS);

                    }
                }
                break;

            case DRIVE_SHOOT_POS_LOAD_ZONE_SIDE_POS_2:
                if (!follower.isBusy()) {
                    telemetry.addLine("Taking LoadZone");
                    hw.intake.setPower(-0.8);
                    hw.helperMotor.setPower(-0.8);
                    follower.followPath(driveShootLongPosLoadPosSideCurve);
                    setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.DRIVE_LOAD_ZONE_SIDE_POS_SHOOT_POS_2);
                }
                break;

            case DRIVE_LOAD_ZONE_SIDE_POS_SHOOT_POS_2:
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    telemetry.addLine("Took LoadZone");

                    hw.helperMotor.setPower(0);
                    follower.followPath(driveLoadPosSideShootLongPos);
                    setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.SHOOT_LZ_3);
                }
                break;

            case SHOOT_LZ_3:
                if (!follower.isBusy() ) {
                    telemetry.addLine("Shot LZ 3");
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        LONG = true;
                        hw.intake.setPower(-0.8);
                        hw.helperMotor.setPower(-0.85);
                        hw.setFlywheels(SHOOTER_POWER_LONG);
                        if (LONG && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_LONG) {
                            hw.shooterServo.setPosition(OPEN_SERVO_POS);
                        } else if (LONG && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_LONG) {
                            hw.shooterServo.setPosition(CLOSE_SERVO_POS);
                        }

                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5) {
                        follower.followPath(driveShootLongPosOffLongPos, true);
                        setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.MOVE_OFF_LONG_LINE);
                        hw.intake.setPower(0);
                        hw.helperMotor.setPower(0);
                        hw.setFlywheels(0);
                        hw.shooterServo.setPosition(CLOSE_SERVO_POS);

                    }
                }
                break;
            case MOVE_OFF_LONG_LINE:
                if (!follower.isBusy()){
                    telemetry.addLine("Done all Paths");

                }
                break;
            default:
                telemetry.addLine("No State Command");
                break;

        }
    }


    public void setPathState(BLUE_LONG_PL_LZ_L3_LZ.PathStateLong newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {

        pathState = BLUE_LONG_PL_LZ_L3_LZ.PathStateLong.DRIVE_START_LONG_POS_SHOOT_LONG_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        hw.shooterServo.setPosition(0);

        buildPaths();
        follower.setPose(startPoseLong);
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
        rightFlywheelSpeed = hw.flywheelR.getVelocity();
        leftFlywheelSpeed =  hw.flywheelL.getVelocity();



        if (SHORT && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_SHORT) {
            hw.shooterServo.setPosition(OPEN_SERVO_POS);
        } else if (SHORT && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_SHORT) {
            hw.shooterServo.setPosition(CLOSE_SERVO_POS);
        }

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());



    }
}
