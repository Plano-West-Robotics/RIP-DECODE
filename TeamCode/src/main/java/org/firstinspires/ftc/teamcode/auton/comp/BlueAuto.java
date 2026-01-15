package org.firstinspires.ftc.teamcode.auton.comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class BlueAuto extends OpMode
{
    public Hardware hardware;
    public Gamepads gamepads;
    public Intake intake;
    public Outtake outtake;
    public AprilTagWebcam webcam;

    public Timer pathTimer;

    public Follower follower;
    public Pose startPose = AutonConstants.mirror(RedAuto.startPose);
    public Pose scorePose = AutonConstants.mirror(RedAuto.scorePose);
    public Pose lineUp1Pose = AutonConstants.mirrorShift(RedAuto.lineUp1Pose, 0, 8);
    public Pose intake1Pose = AutonConstants.mirrorShift(RedAuto.intake1Pose, 0, 8);
    public Pose leave1Pose = AutonConstants.mirror(RedAuto.leave1Pose);

    public Path preloadPath, lineUp1Path, intake1Path, intermediatePath, score1Path, leave1Path;

    public PathState pathState = PathState.START;

    public int numShot = 0;

    public enum PathState
    {
        START,
        TO_PRELOAD_SCORE,
        AT_PRELOAD_SCORE,
        TO_LINEUP1,
        TO_INTAKE1,
        TO_INTERMEDIATE1,
        TO_SCORE1,
        AT_SCORE1,
        LEAVE_LINE,
        STOP
    }

    @Override
    public void init()
    {
        hardware = new Hardware(hardwareMap);
        gamepads = new Gamepads(gamepad1, gamepad2);
        intake = new Intake(hardware);
        outtake = new Outtake(hardware);
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.BLUE_GOAL_ID);

        pathTimer = new Timer();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop()
    {
        if (gamepads.justPressed(Button.GP1_A)) webcam.toggleGoalId();

        boolean cameraIsReady = webcam.portal.getCameraState() == VisionPortal.CameraState.STREAMING;
        telemetry.addData("Camera Is Ready?", cameraIsReady);
        telemetry.addData("Goal Color", webcam.getGoalId() == AprilTagWebcam.RED_GOAL_ID ? "RED" : "BLUE");

        gamepads.update(gamepad1, gamepad2);
        telemetry.update();
    }

    @Override
    public void loop()
    {
        follower.update();
        autonomousPaths();
    }

    public void buildPaths()
    {
        preloadPath = new Path(new BezierLine(startPose, scorePose));
        preloadPath.setConstantHeadingInterpolation(startPose.getHeading());

        lineUp1Path = new Path(new BezierLine(scorePose, lineUp1Pose));
        lineUp1Path.setLinearHeadingInterpolation(scorePose.getHeading(), lineUp1Pose.getHeading());

        intake1Path = new Path(new BezierLine(lineUp1Pose, intake1Pose));
        intake1Path.setConstantHeadingInterpolation(intake1Pose.getHeading());
        intake1Path.setVelocityConstraint(AutonConstants.INTAKE_1_VEL_CONSTRAINT);

        intermediatePath = new Path(new BezierLine(intake1Pose, lineUp1Pose));
        intermediatePath.setConstantHeadingInterpolation(intake1Pose.getHeading());

        score1Path = new Path(new BezierLine(lineUp1Pose, scorePose));
        score1Path.setLinearHeadingInterpolation(lineUp1Pose.getHeading(), scorePose.getHeading());

        leave1Path = new Path(new BezierLine(scorePose, leave1Pose));
        leave1Path.setConstantHeadingInterpolation(scorePose.getHeading());
    }

    public void autonomousPaths()
    {
        telemetry.addData("Path State", pathState);

        switch (pathState)
        {
            case START:
                follower.followPath(preloadPath);
                pathState = PathState.TO_PRELOAD_SCORE;
                break;
            case TO_PRELOAD_SCORE:
                if (!follower.isBusy())
                {
                    ((DcMotorEx) outtake.motor.motor).setVelocity(Outtake.MANUAL_ANGULAR_RATE);
                    pathState = PathState.AT_PRELOAD_SCORE;
                    pathTimer.resetTimer();
                }
                break;
            case AT_PRELOAD_SCORE:
                shoot();
                if (pathTimer.getElapsedTimeSeconds() > AutonConstants.SHOOT_THREE_BALLS_SECONDS)
                {
                    intake.forwardRegular();
                    outtake.motor.setPower(0);
                    follower.followPath(lineUp1Path);
                    pathState = PathState.TO_LINEUP1;
                }
                break;
            case TO_LINEUP1:
                if (!follower.isBusy())
                {
                    outtake.motor.setPower(0);
                    follower.followPath(intake1Path);
                    pathState = PathState.TO_INTAKE1;
                }
                break;
            case TO_INTAKE1:
                if (!follower.isBusy())
                {
                    intake.stop();
                    follower.followPath(intermediatePath);
                    pathState = PathState.TO_INTERMEDIATE1;
                }
                break;
            case TO_INTERMEDIATE1:
                if (!follower.isBusy())
                {
                    follower.followPath(score1Path);
                    pathState = PathState.TO_SCORE1;
                }
                break;
            case TO_SCORE1:
                if (!follower.isBusy())
                {
                    pathState = PathState.AT_SCORE1;
                    pathTimer.resetTimer();
                }
                break;
            case AT_SCORE1:
                shoot();
                if (pathTimer.getElapsedTimeSeconds() > AutonConstants.SHOOT_THREE_BALLS_SECONDS)
                {
                    intake.forwardRegular();
                    outtake.motor.setPower(0);
                    follower.followPath(leave1Path);
                    pathState = PathState.LEAVE_LINE;
                }
                break;
            case LEAVE_LINE:
                if (!follower.isBusy())
                {
                    outtake.motor.setPower(0);
                    pathState = PathState.STOP;
                }
                break;
            case STOP:
                ((DcMotorEx) outtake.motor.motor).setVelocity(0);
                break;
        }
    }

    public void shoot()
    {
        AprilTagDetection detection = webcam.getGoalDetection();
        if (detection != null)
        {
            webcam.updateRange(detection.ftcPose.range);
            double targetAngularRate = Outtake.toAngularRate(Outtake.calculateIdealFlywheelTangentialVelocity(webcam.getRange()));
            ((DcMotorEx) outtake.motor.motor).setVelocity(targetAngularRate);
            double error = ((DcMotorEx) outtake.motor.motor).getVelocity() - targetAngularRate;

            if (Math.abs(error) < Outtake.ANGULAR_RATE_ERROR_TOLERANCE)
            {
                intake.forwardLaunch();
            }
            else
            {
                intake.stop();
            }

            telemetry.addData("Range", webcam.getRange());
            telemetry.addData("Target Angular Rate", targetAngularRate);
            telemetry.addData("Error", error);
        }
    }
}
