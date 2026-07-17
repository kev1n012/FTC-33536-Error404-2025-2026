package org.firstinspires.ftc.teamcode.Autonomous.Blue.Long;

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
public class LONG_PL_L3 extends OpMode {


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





    LONG_PL_LZ_L3_LZ.PathStateLong pathState;

    private final Pose startPoseLong = new Pose(55.5,8, Math.toRadians(90));
    private final Pose shootPoseLong = new Pose(55.5,15, Math.toRadians(112));
    private final Pose LoadingZoneUpPose = new Pose(16,16,Math.toRadians(270));
    private final Pose LoadingZoneUpCurve = new Pose(13,33);
    private final Pose LoadingZoneSidePose = new Pose(16,16, Math.toRadians(180));
    private final Pose LoadingZoneSideCurve = new Pose(52, 7.5);
    private final Pose line3LongStart = new Pose(55.5,15, Math.toRadians(115));
    private final Pose line3LongCurve = new Pose(60, 36);
    private final Pose line3LongEnd = new Pose(16, 41, Math.toRadians(180));
    private final Pose offPoseLong = new Pose(59,36, Math.toRadians(90));



    private PathChain driveStartLongPosShootLongPos;
    private PathChain driveShootLongPosStartL3;
    private PathChain driveStartLongL3EndL3;
    private PathChain driveEndL3ShootLongPose;
    private PathChain driveShootLongPosOffLongPos;

    public void buildPaths() {
        driveStartLongPosShootLongPos = follower.pathBuilder()
                .addPath(new BezierLine(startPoseLong,shootPoseLong))
                .setLinearHeadingInterpolation(startPoseLong.getHeading(), shootPoseLong.getHeading())
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
        driveShootLongPosOffLongPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseLong,offPoseLong))
                .setLinearHeadingInterpolation(shootPoseLong.getHeading(), offPoseLong.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_START_LONG_POS_SHOOT_LONG_POS:
                follower.followPath(driveStartLongPosShootLongPos, true);
                setPathState(LONG_PL_LZ_L3_LZ.PathStateLong.SHOOT_PRELOAD_LONG);
                flywheelR.setVelocity(ShooterVel());
                flywheelL.setVelocity(ShooterVel());
                break;
            case SHOOT_PRELOAD_LONG:
                if (!follower.isBusy() ) {
                    telemetry.addLine("Done Path 1");
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        LONG = true;
                        intake.setPower(-0.8);
                        helper_motor.setPower(-0.7);
                        flywheelL.setPower(SHOOTER_POWER_LONG);
                        flywheelR.setPower(SHOOTER_POWER_LONG);
                        if (LONG && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_LONG) {
                            shooter_servo.setPosition(OPEN_SERVO_POS);
                        } else if (LONG && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_LONG) {
                            shooter_servo.setPosition(CLOSE_SERVO_POS);
                        }

                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5) {
                        setPathState(LONG_PL_LZ_L3_LZ.PathStateLong.DRIVE_TAKE_LINE_3_LONG);
                        intake.setPower(0);
                        helper_motor.setPower(0);
                        flywheelR.setPower(ShooterVel());
                        flywheelL.setPower(ShooterVel());
                        shooter_servo.setPosition(CLOSE_SERVO_POS);

                    }
                }
                break;

            case DRIVE_TAKE_LINE_3_LONG:
                if (!follower.isBusy() ) {
                    telemetry.addLine("Taking Line 1");
                    intake.setPower(-0.8);
                    helper_motor.setPower(-0.8);
                    follower.followPath(driveStartLongL3EndL3);
                    setPathState(LONG_PL_LZ_L3_LZ.PathStateLong.DRIVE_LINE_3_END_SHOOT_LONG_POS);
                }
                break;
            case DRIVE_LINE_3_END_SHOOT_LONG_POS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {

                    helper_motor.setPower(0);
                    telemetry.addLine("Shooting Line 1");
                    follower.followPath(driveEndL3ShootLongPose);
                    setPathState(LONG_PL_LZ_L3_LZ.PathStateLong.SHOOT_LINE_3_LONG);
                }
                break;
            case SHOOT_LINE_3_LONG:
                if (!follower.isBusy()) {
                    telemetry.addLine("Shot Line 1");
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        intake.setPower(-0.8);
                        helper_motor.setPower(-0.7);
                        flywheelL.setPower(SHOOTER_POWER_LONG);
                        flywheelR.setPower(SHOOTER_POWER_LONG);
                        if (LONG && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_LONG) {
                            shooter_servo.setPosition(OPEN_SERVO_POS);
                        } else if (LONG && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_LONG) {
                            shooter_servo.setPosition(CLOSE_SERVO_POS);
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5) {
                        follower.followPath(driveShootLongPosOffLongPos, true);
                        setPathState(LONG_PL_LZ_L3_LZ.PathStateLong.MOVE_OFF_LONG_LINE);
                        intake.setPower(0);
                        helper_motor.setPower(0);
                        flywheelR.setPower(0);
                        flywheelL.setPower(0);
                        shooter_servo.setPosition(CLOSE_SERVO_POS);
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


    public void setPathState(LONG_PL_LZ_L3_LZ.PathStateLong newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {

        pathState = LONG_PL_LZ_L3_LZ.PathStateLong.DRIVE_START_LONG_POS_SHOOT_LONG_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        configureMotors();
        shooter_servo.setPosition(0);

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
        rightFlywheelSpeed = flywheelR.getVelocity();
        leftFlywheelSpeed =  flywheelL.getVelocity();



        if (SHORT && rightFlywheelSpeed > OPEN_SHOOTER_SERVO_SHORT) {
            shooter_servo.setPosition(OPEN_SERVO_POS);
        } else if (SHORT && rightFlywheelSpeed < CLOSE_SHOOTER_SERVO_SHORT) {
            shooter_servo.setPosition(CLOSE_SERVO_POS);
        }

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());



    }
}
