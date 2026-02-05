package org.firstinspires.ftc.teamcode.teleop.comp;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.control.Analog;
import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.teamcode.teleop.tune.DashboardWebcamBearingPIDFTuner;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(group = "Comp")
public class MainComp extends BaseTeleOp
{
    public enum State
    {
        STANDBY,
        MANUAL_SHOOTING,
        WEBCAM_SHOOTING
    }

    public FieldCentricDrive drive;
    public Intake intake;
    public Outtake outtake;
    public AprilTagWebcam webcam;

    public PIDFController bearingController;
    public StateMachine fsm;

    public AprilTagDetection lastValidDetection;

    double oldTime = 0;

    @Override
    public void setup()
    {
        drive = new FieldCentricDrive(hardware);
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
            .state(State.STANDBY)
            .onEnter(() -> outtake.motor.setPower(0))
            .loop(() -> {
                drive.update(gamepads);
                intake.update(gamepads);
                outtake.update(gamepads);
                outtake.motor.setPower(0);
                webcam.update(gamepads);
            })
            .transition(() ->
                gamepads.exceedsThreshold(Analog.GP1_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD)
                && outtake.getMode() == Outtake.ControlMode.MANUAL_CONTROL,
                State.MANUAL_SHOOTING
            )
            .transition(() ->
                    gamepads.exceedsThreshold(Analog.GP1_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD)
                        && outtake.getMode() == Outtake.ControlMode.WEBCAM_CONTROL,
                State.WEBCAM_SHOOTING
            )
            .onExit(() -> intake.stop())

            .state(State.MANUAL_SHOOTING)
            .onEnter(() -> ((DcMotorEx) outtake.motor.motor).setVelocity(Outtake.MANUAL_ANGULAR_RATE))
            .loop(() -> {
                drive.update(gamepads);
                ((DcMotorEx) outtake.motor.motor).setVelocity(Outtake.MANUAL_ANGULAR_RATE);
                webcam.update(gamepads);

                double error = ((DcMotorEx) outtake.motor.motor).getVelocity() - Outtake.MANUAL_ANGULAR_RATE;
                telemetry.addData("Error", error);

                if (gamepads.isPressed(Button.GP1_A))
                {
                    if (Math.abs(error) < Outtake.NORMAL_ERROR_TOLERANCE_TPS)
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
                State.STANDBY
            )
            .onExit(() -> {
                intake.forwardRegular();
                intake.stop();
                outtake.motor.setPower(0);
            })

            .state(State.WEBCAM_SHOOTING)
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

                    ((DcMotorEx) outtake.motor.motor).setVelocity(0);

                    telemetry.addData("AprilTag Found", false);
                    intake.stop();
                    return;
                }

                telemetry.addData("AprilTag Currently Found", detection != null);

                webcam.updateBearing(detectionToUse.ftcPose.bearing);
                webcam.updateRange(detectionToUse.ftcPose.range);

                double rx = 0;
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

                double targetAngularRate = Outtake.toAngularRate(Outtake.calculateIdealFlywheelTangentialVelocity(webcam.getRange()));

                drive.drive(gamepads.getAnalogValue(Analog.GP1_LEFT_STICK_Y), gamepads.getAnalogValue(Analog.GP1_LEFT_STICK_X), rx);

                DcMotorEx flywheel = (DcMotorEx) outtake.motor.motor;
                flywheel.setVelocity(targetAngularRate);

                double error = flywheel.getVelocity() - targetAngularRate;

                telemetry.addData("Range", webcam.getRange());
                telemetry.addData("Bearing", webcam.getBearing());
                telemetry.addData("RX", rx);
                telemetry.addData("Target Angular Rate", targetAngularRate);
                telemetry.addData("Error", error);

                if (gamepads.isPressed(Button.GP1_A))
                {
                    if (Math.abs(error) < Outtake.NORMAL_ERROR_TOLERANCE_TPS)
                    {
                        intake.forwardLaunch();
                        gamepad1.stopRumble();
                    }
                    else
                    {
                        intake.stop();
                        gamepad1.rumbleBlips(1);
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
                State.STANDBY
            )
            .onExit(() -> {
                intake.forwardRegular();
                intake.stop();
                outtake.motor.setPower(0);
            })
            .build();
    }

    @Override
    public void init_loop()
    {
        if (gamepads.justPressed(Button.GP1_A))
        {
            webcam.toggleGoalId();
        }
        telemetry.addData("Goal Color", webcam.getGoalId() == AprilTagWebcam.RED_GOAL_ID ? "RED" : "BLUE");

        boolean cameraIsReady = webcam.portal.getCameraState() == VisionPortal.CameraState.STREAMING;
        telemetry.addData("Camera Is Ready?", cameraIsReady);
        telemetry.update();
    }

    @Override
    public void start()
    {
        fsm.start();
        drive.resetHeading();
    }

    @Override
    public void run()
    {
        fsm.update();
        telemetry.addData("Current State", fsm.getState());
        telemetry.addData("Threshold Exceeded", gamepads.exceedsThreshold(Analog.GP1_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD));
        telemetry.addData("Outtake Mode", outtake.getMode());
        telemetry.addData("Goal Color", webcam.getGoalId() == AprilTagWebcam.RED_GOAL_ID ? "RED" : "BLUE");
        telemetry.addData("Outtake Motor Angular Velocity (ticks/sec)", ((DcMotorEx) outtake.motor.motor).getVelocity());
        telemetry.addData("Outtake Motor Angular Velocity (rev/min)", ((DcMotorEx) outtake.motor.motor).getVelocity() * 60 * (1 / Outtake.TICKS_PER_REVOLUTION));
        telemetry.addData("Outtake Motor Current Draw (Amperes)", ((DcMotorEx) outtake.motor.motor).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Pinpoint IMU Yaw (Degrees)", Math.toDegrees(drive.getHeading()));
        telemetry.addData("Field Centric Drive is Functional", (Math.pow(Math.toDegrees(drive.getHeading()), 2) > 0) ? "YES" : "NO");

        // for checking pinpoint loop times & status

        telemetry.addData("Status", hardware.pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", hardware.pinpoint.getFrequency());

        double newTime = getRuntime();
        double period = newTime - oldTime;
        double frequency = 1 / period;
        oldTime = newTime;
        telemetry.addData("REV Hub Frequency", frequency);
    }
}
