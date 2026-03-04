package org.firstinspires.ftc.teamcode.teleop.comp;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.core.control.Analog;
import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.tune.custom.DashboardWebcamBearingPIDFTuner;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class ExampleTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    public Hardware hardware;
    public Gamepads gamepads;

    public FieldCentricDrive drive;
    public Intake intake;
    public Outtake outtake;
    public AprilTagWebcam webcam;

    public PIDFController bearingController;
    public StateMachine fsm;

    public AprilTagDetection lastValidDetection;
    private Pose redGoalPose = new Pose(-1, -1);
    private Pose blueGoalPose = new Pose(-1, -1);

    public enum State
    {
        STANDBY,
        MANUAL_SHOOTING,
        WEBCAM_SHOOTING
    }


    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
        gamepads = new Gamepads(gamepad1, gamepad2);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardware);
        outtake = new Outtake(hardware);
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);

        bearingController = new PIDFController(
                DashboardWebcamBearingPIDFTuner.P,
                DashboardWebcamBearingPIDFTuner.I,
                DashboardWebcamBearingPIDFTuner.D,
                DashboardWebcamBearingPIDFTuner.F
        );
        fsm = new StateMachineBuilder()
                .state(MainComp.State.STANDBY)
                .onEnter(() -> outtake.stop())
                .loop(() -> {
                    drive.update(gamepads);
                    intake.update(gamepads);
                    outtake.update(gamepads);
                    outtake.setVelocity(-400);
                    webcam.update(gamepads);
                    outtake.hoodDown();
                })
                .transition(() ->
                                gamepads.exceedsThreshold(Analog.GP1_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD)
                                        && outtake.getMode() == Outtake.ControlMode.MANUAL_CONTROL,
                        MainComp.State.MANUAL_SHOOTING
                )
                .transition(() ->
                                gamepads.exceedsThreshold(Analog.GP1_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD)
                                        && outtake.getMode() == Outtake.ControlMode.WEBCAM_CONTROL,
                        MainComp.State.WEBCAM_SHOOTING
                )
                .onExit(() -> {
                    intake.stop();
                })


                .state(MainComp.State.MANUAL_SHOOTING)
                .onEnter(() -> outtake.setVelocity(Outtake.MANUAL_ANGULAR_RATE))
                .loop(() -> {
                    drive.update(gamepads);
                    outtake.setVelocity(Outtake.MANUAL_ANGULAR_RATE);
                    webcam.update(gamepads);

                    double error = outtake.getAverageVelocity() - Outtake.MANUAL_ANGULAR_RATE;
                    telemetry.addData("Error", error);

                    if (gamepads.isPressed(Button.GP1_A))
                    {
                        if (Math.abs(error) < Outtake.NORMAL_ERROR_TOLERANCE)
                        {
                            intake.forwardLaunch();
                            gamepad1.stopRumble();
                        }
                        else
                        {
                            intake.stop();
                            if (!gamepad1.isRumbling())
                            {
                                gamepad1.rumble(500);
                            }
                        }
                    }
                    else
                    {
                        intake.stop();
                        gamepad1.stopRumble();
                    }
                })
                .transition(() ->
                                !gamepads.exceedsThreshold(Analog.GP1_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD),
                        MainComp.State.STANDBY
                )
                .onExit(() -> {
                    intake.forwardRegular();
                    intake.stop();
                    outtake.stop();
                    if (!gamepad2.isRumbling())
                        gamepad2.rumble(1000);
                })

                .state(MainComp.State.WEBCAM_SHOOTING)
                .loop(() -> {
                    AprilTagDetection detection = webcam.getGoalDetection();

                    AprilTagDetection detectionToUse = (detection != null) ? detection : lastValidDetection;

                    if (detectionToUse == null)
                    {
                        if (!gamepad1.isRumbling())
                        {
                            gamepad1.rumble(1000);
                        }
                        drive.update(gamepads);

                        outtake.setVelocity(Outtake.MANUAL_ANGULAR_RATE);

                        telemetry.addData("AprilTag Found", false);
                        intake.stop();
                        return;
                    }

                    telemetry.addData("AprilTag Currently Found", detection != null);

                    webcam.updateBearing(detectionToUse.ftcPose.bearing);
                    webcam.updateRange(detectionToUse.ftcPose.range);

                    double rx = gamepads.getAnalogValue(Analog.GP1_RIGHT_STICK_X);
                    if (detection != null)
                    {
                        webcam.updateBearing(detection.ftcPose.bearing);
                        rx = bearingController.calculate(webcam.getBearing(), 0);
                    }
                    else
                    {
                        // for telemetry
                        webcam.updateBearing(lastValidDetection.ftcPose.bearing);
                    }

                    double targetAngularRate = Outtake.MANUAL_ANGULAR_RATE;

                    drive.drive(gamepads.getAnalogValue(Analog.GP1_LEFT_STICK_Y), gamepads.getAnalogValue(Analog.GP1_LEFT_STICK_X), rx);

                    outtake.setVelocity(targetAngularRate);

                    double error = outtake.getAverageVelocity() - targetAngularRate;

                    telemetry.addData("Range", webcam.getRange());
                    telemetry.addData("Bearing", webcam.getBearing());
                    telemetry.addData("RX", rx);
                    telemetry.addData("Target Angular Rate", targetAngularRate);
                    telemetry.addData("Error", error);

                    if (gamepads.isPressed(Button.GP1_A))
                    {
                        if (Math.abs(error) < Outtake.NORMAL_ERROR_TOLERANCE)
                        {
                            intake.forwardLaunch();
                            gamepad1.stopRumble();
                        }
                        else
                        {
                            intake.stop();
                            if (!gamepad1.isRumbling())
                            {
                                gamepad1.rumble(500);
                            }
                        }
                    }
                    else
                    {
                        intake.stop();
                        gamepad1.stopRumble();
                    }

                    if (detection != null)
                    {
                        lastValidDetection = detection;
                    }
                })
                .transition(() ->
                                !gamepads.exceedsThreshold(Analog.GP1_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD),
                        MainComp.State.STANDBY
                )
                .onExit(() -> {
                    intake.forwardRegular();
                    //intake.stop();
                    outtake.stop();
                    if (!gamepad2.isRumbling())
                        gamepad2.rumble(1000);
                })
                .build();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        fsm.start();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        fsm.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Field Centric
            );

            //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false // Field Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }



    public double calculateRedAngleInRadians()
    {
        double yPosDifference = redGoalPose.getY() - follower.getPose().getY();
        double xPosDifference = redGoalPose.getX() - follower.getPose().getX();
        return Math.atan2(yPosDifference, xPosDifference);
    }
}

