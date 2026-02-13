//package org.firstinspires.ftc.teamcode.auton.comp;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import com.sfdev.assembly.state.StateMachine;
//import com.sfdev.assembly.state.StateMachineBuilder;
//
//import org.firstinspires.ftc.teamcode.core.control.Button;
//import org.firstinspires.ftc.teamcode.core.control.Gamepads;
//import org.firstinspires.ftc.teamcode.hardware.Hardware;
//import org.firstinspires.ftc.teamcode.hardware.RightStopper;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
//import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//@Autonomous(preselectTeleOp = "MainComp", group = "Comp")
//public class NewAuto extends OpMode
//{
//    public Pose startPose = AutonConstants.RED_START;
//    public Pose scorePose = AutonConstants.RED_SCORE;
//    public Pose lineUp1Pose = AutonConstants.RED_LINEUP_1;
//    public Pose intake1Pose = AutonConstants.RED_INTAKE_1;
//    public Pose leave1Pose = AutonConstants.RED_LEAVE_1;
//
//    public Hardware hardware;
//    public RightStopper rightStopper;
//    public Gamepads gamepads;
//    public Intake intake;
//    public Outtake outtake;
//    public AprilTagWebcam webcam;
//
//    public Timer pathTimer;
//
//    public Follower follower;
//
//    public Path preloadPath, lineUp1Path, intake1Path, score1Path, leave1Path;
//
//    public int numShot = 0;
//
//    public boolean isRed = true;
//    public boolean score1ReverseLaunchDone = false;
//
//    public enum Score1Phase {WAIT, REVERSE, DONE}
//    Score1Phase score1Phase;
//
//    public StateMachine fsm;
//
//    public enum PathState
//    {
//        START,
//        AT_PRELOAD_SCORE,
//        TO_LINEUP1,
//        TO_INTAKE1,
//        TO_SCORE1,
//        AT_SCORE1,
//        LEAVE_LINE,
//        STOP
//    }
//
//    @Override
//    public void init()
//    {
//        hardware = new Hardware(hardwareMap);
//        rightStopper = hardware.rightStopper;
//        gamepads = new Gamepads(gamepad1, gamepad2);
//        intake = new Intake(hardware);
//        outtake = new Outtake(hardware);
//        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
//
//        pathTimer = new Timer();
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        buildPaths();
//        buildMachine();
//
//        Drawing.init();
//    }
//
//    @Override
//    public void init_loop()
//    {
//        if (gamepads.justPressed(Button.GP1_A))
//        {
//            webcam.toggleGoalId();
//
//            if (isRed)
//            {
//                startPose = AutonConstants.BLUE_START;
//                scorePose = AutonConstants.BLUE_SCORE;
//                lineUp1Pose = AutonConstants.BLUE_LINEUP_1;
//                intake1Pose = AutonConstants.BLUE_INTAKE_1;
//                leave1Pose = AutonConstants.BLUE_LEAVE_1;
//            }
//            else
//            {
//                startPose = AutonConstants.RED_START;
//                scorePose = AutonConstants.RED_SCORE;
//                lineUp1Pose = AutonConstants.RED_LINEUP_1;
//                intake1Pose = AutonConstants.RED_INTAKE_1;
//                leave1Pose = AutonConstants.RED_LEAVE_1;
//            }
//
//            isRed = !isRed;
//        }
//
//        boolean cameraIsReady = webcam.portal.getCameraState() == VisionPortal.CameraState.STREAMING;
//        telemetry.addData("Camera Is Ready?", cameraIsReady);
//        telemetry.addData("Goal Color", webcam.getGoalId() == AprilTagWebcam.RED_GOAL_ID ? "RED" : "BLUE");
//
//        gamepads.update(gamepad1, gamepad2);
//        telemetry.update();
//    }
//
//    @Override
//    public void start()
//    {
//        fsm.start();
//    }
//
//    @Override
//    public void loop()
//    {
//        follower.update();
//        Drawing.drawDebug(follower);
//
//        boolean machineExists = fsm != null;
//        if (machineExists) fsm.update();
//
//        telemetry.addData("Path State", machineExists ? fsm.getStateEnum() : "(no machine)");
//        telemetry.update();
//    }
//
//    public void buildPaths()
//    {
//        preloadPath = new Path(new BezierLine(startPose, scorePose));
//        preloadPath.setConstantHeadingInterpolation(startPose.getHeading());
//
//        lineUp1Path = new Path(new BezierLine(scorePose, lineUp1Pose));
//        lineUp1Path.setLinearHeadingInterpolation(scorePose.getHeading(), lineUp1Pose.getHeading());
//
//        intake1Path = new Path(new BezierLine(lineUp1Pose, intake1Pose));
//        intake1Path.setConstantHeadingInterpolation(intake1Pose.getHeading());
//        intake1Path.setVelocityConstraint(AutonConstants.INTAKE_1_VEL_CONSTRAINT);
//
//        score1Path = new Path(new BezierLine(intake1Pose, scorePose));
//        score1Path.setLinearHeadingInterpolation(intake1Pose.getHeading(), scorePose.getHeading());
//
//        leave1Path = new Path(new BezierLine(scorePose, leave1Pose));
//        leave1Path.setConstantHeadingInterpolation(scorePose.getHeading());
//    }
//
//    public void buildMachine()
//    {
//        score1ReverseLaunchDone = false;
//        score1Phase = Score1Phase.WAIT;
//
//        fsm = new StateMachineBuilder()
//
//            // drive to preload score while spinning up
//            .state(PathState.START)
//            .onEnter(() -> {
//                rightStopper.go();
//                follower.followPath(preloadPath);
//            })
//            .loop(() -> {
//                ((DcMotorEx) outtake.motor.motor).setVelocity(Outtake.MANUAL_ANGULAR_RATE);
//            })
//            .transition(() -> !follower.isBusy(), PathState.AT_PRELOAD_SCORE)
//
//            // shoot the balls
//            .state(PathState.AT_PRELOAD_SCORE)
//            .onEnter(() -> pathTimer.resetTimer())
//            .loop(() -> shoot(Outtake.NORMAL_ERROR_TOLERANCE))
//            .transition(() -> pathTimer.getElapsedTimeSeconds() > AutonConstants.PRELOAD_SCORE_TIME,
//                PathState.TO_LINEUP1,
//                () -> {
//                    // turn on the intake, run the flywheel in reverse to prevent the balls from jumping out
//                    intake.forwardRegular();
//                    ((DcMotorEx) outtake.motor.motor).setVelocity(-800);
//                    follower.followPath(lineUp1Path);
//                }
//            )
//
//            .state(PathState.TO_LINEUP1)
//            .transition(() -> !follower.isBusy(), PathState.TO_INTAKE1,
//                () -> {
//                    rightStopper.stop();
//                    follower.followPath(intake1Path);
//                }
//            )
//
//            .state(PathState.TO_INTAKE1)
//            .transition(() -> !follower.isBusy(), PathState.TO_SCORE1,
//                () -> {
//                    follower.followPath(score1Path);
//                    pathTimer.resetTimer();
//                    score1Phase = Score1Phase.WAIT;
//                }
//            )
//
//            .state(PathState.TO_SCORE1)
//            .loop(() -> {
//                // phase WAIT: after DISABLE_INTAKE_SECONDS, reverse intake for a small period of time and spin up the flywheel on the way to the goal
//                if (score1Phase == Score1Phase.WAIT && pathTimer.getElapsedTimeSeconds() >= AutonConstants.DISABLE_INTAKE_SECONDS)
//                {
//                    ((DcMotorEx) outtake.motor.motor).setVelocity(Outtake.MANUAL_ANGULAR_RATE);
//                    intake.reverseLaunch();
//                    rightStopper.go();
//
//                    score1Phase = Score1Phase.REVERSE;
//                    pathTimer.resetTimer();
//                }
//
//                // phase REVERSE: after REVERSE_INTAKE_SECONDS, stop intake
//                if (score1Phase == Score1Phase.REVERSE && pathTimer.getElapsedTimeSeconds() >= AutonConstants.REVERSE_INTAKE_SECONDS)
//                {
//                    intake.stop();
//                    score1Phase = Score1Phase.DONE;
//                }
//            })
//            .transition(() -> !follower.isBusy(), PathState.AT_SCORE1,
//                () -> {
//                    intake.reverseLaunch();
//                }
//            )
//
//            .state(PathState.AT_SCORE1)
//            .onEnter(() -> {
//                pathTimer.resetTimer();
//                score1ReverseLaunchDone = false;
//            })
//            .loop(() -> {
//                if (!score1ReverseLaunchDone && pathTimer.getElapsedTimeSeconds() >= AutonConstants.REVERSE_INTAKE_SECONDS)
//                {
//                    intake.stop();
//                    score1ReverseLaunchDone = true;
//                    pathTimer.resetTimer();
//                }
//
//                rightStopper.go();
//                shoot(Outtake.NORMAL_ERROR_TOLERANCE);
//            })
//            .transition(() -> pathTimer.getElapsedTimeSeconds() > (AutonConstants.FIRST_THREE_SCORE_TIME - AutonConstants.REVERSE_INTAKE_SECONDS),
//                PathState.LEAVE_LINE,
//                () -> {
//                    intake.stop();
//                    ((DcMotorEx) outtake.motor.motor).setVelocity(0);
//                    follower.followPath(leave1Path, true);
//                }
//            )
//
//            .state(PathState.LEAVE_LINE)
//            .transition(() -> !follower.isBusy(), PathState.STOP,
//                () -> ((DcMotorEx) outtake.motor.motor).setVelocity(0)
//            )
//
//            .state(PathState.STOP)
//            .loop(() -> ((DcMotorEx) outtake.motor.motor).setVelocity(0))
//
//            .build();
//    }
//
//    public void shoot(double tolerance)
//    {
//        AprilTagDetection detection = webcam.getGoalDetection();
//        if (detection != null)
//        {
//            webcam.updateRange(detection.ftcPose.range);
//            double targetAngularRate = Outtake.toAngularRate(Outtake.calculateIdealFlywheelTangentialVelocity(webcam.getRange()));
//            ((DcMotorEx) outtake.motor.motor).setVelocity(targetAngularRate);
//            double error = ((DcMotorEx) outtake.motor.motor).getVelocity() - targetAngularRate;
//
//            if (Math.abs(error) < tolerance)
//            {
//                intake.forwardLaunch();
//            }
//            else
//            {
//                intake.stop();
//            }
//
//            telemetry.addData("Range", webcam.getRange());
//            telemetry.addData("Target Angular Rate", targetAngularRate);
//            telemetry.addData("Error", error);
//        }
//    }
//}
