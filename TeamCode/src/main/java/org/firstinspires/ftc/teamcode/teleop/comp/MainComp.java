package org.firstinspires.ftc.teamcode.teleop.comp;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.core.control.Analog;
import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.teamcode.teleop.tune.DashboardWebcamAngularPIDFTuner;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(group = "Comp")
public class MainComp extends BaseTeleOp
{
    public enum State
    {
        STANDBY,
        MANUAL_SHOOTING,
//        WEBCAM_SHOOTING
    }

    public AbstractDrive drive;
    public Intake intake;
    public Outtake outtake;
//    public AprilTagWebcam webcam;

    public PIDFController controller;
    public StateMachine fsm;

    @Override
    public void setup()
    {
        drive = new FieldCentricDrive(hardware);
        intake = new Intake(hardware);
        outtake = new Outtake(hardware);
//        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);

        controller = new PIDFController(
            DashboardWebcamAngularPIDFTuner.P,
            DashboardWebcamAngularPIDFTuner.I,
            DashboardWebcamAngularPIDFTuner.D,
            DashboardWebcamAngularPIDFTuner.F
        );
        fsm = new StateMachineBuilder()
            .state(State.STANDBY)
            .onEnter(() ->
            {
                ((FieldCentricDrive) drive).imu.resetYaw();
                outtake.motor.setPower(0);
            })
            .loop(() -> {
                drive.update(gamepads);
                intake.update(gamepads);
                outtake.update(gamepads);
                outtake.motor.setPower(0);
//                webcam.update(gamepads);
            })
            .transition(() ->
                gamepads.exceedsThreshold(Analog.GP1_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD)
                && outtake.getMode() == Outtake.ControlMode.MANUAL_CONTROL,
                State.MANUAL_SHOOTING
            )
//            .transition(() ->
//                    gamepads.exceedsThreshold(Analog.GP1_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD)
//                        && outtake.getMode() == Outtake.ControlMode.WEBCAM_CONTROL,
//                State.WEBCAM_SHOOTING
//            )
            .onExit(() -> intake.stop())

            .state(State.MANUAL_SHOOTING)
            .onEnter(() -> ((DcMotorEx) outtake.motor.motor).setVelocity(Outtake.MANUAL_ANGULAR_RATE))
            .loop(() -> {
                drive.update(gamepads);
//                webcam.update(gamepads);

                double error = ((DcMotorEx) outtake.motor.motor).getVelocity() - Outtake.MANUAL_ANGULAR_RATE;
                telemetry.addData("Error", error);

                if (gamepads.isPressed(Button.GP1_A))
                {
                    if (Math.abs(error) < Outtake.ANGULAR_RATE_ERROR_TOLERANCE)
                    {
                        intake.forwardLaunch();
                        gamepad1.stopRumble();
                    }
                    else
                    {
                        intake.stop();
                        if (!gamepad1.isRumbling())
                        {
                            gamepad1.rumble(1000);
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

//            .state(State.WEBCAM_SHOOTING)
//            .loop(() -> {
//                AprilTagDetection detection = webcam.getGoalDetection();
//                if (detection == null)
//                {
//                    if (!gamepad1.isRumbling())
//                    {
//                        gamepad1.rumble(1000);
//                    }
//                    drive.update(gamepads);
//                    outtake.motor.setPower(0);
//                }
//                else
//                {
//                    double bearing = detection.ftcPose.bearing;
//                    double range = detection.ftcPose.range;
//
//                    double rx = controller.calculate(bearing, 0);
//                    double targetAngularRate = Outtake.toAngularRate(Outtake.calculateIdealFlywheelTangentialVelocity(range));
//
//                    drive.drive(0, 0, rx);
//                    ((DcMotorEx) outtake.motor.motor).setVelocity(targetAngularRate);
//
//                    double error = ((DcMotorEx) outtake.motor.motor).getVelocity() - targetAngularRate;
//
//                    telemetry.addData("Range", range);
//                    telemetry.addData("Bearing", bearing);
//                    telemetry.addData("RX", rx);
//                    telemetry.addData("Target Angular Rate", targetAngularRate);
//
//                    if (gamepads.isPressed(Button.GP1_A))
//                    {
//                        if (Math.abs(error) < Outtake.ANGULAR_RATE_ERROR_TOLERANCE)
//                        {
//                            intake.forwardLaunch();
//                            gamepad1.stopRumble();
//                        }
//                        else
//                        {
//                            intake.stop();
//                            gamepad1.rumbleBlips(1);
//                        }
//                    }
//                    else
//                    {
//                        intake.stop();
//                        gamepad1.stopRumble();
//                    }
//                }
//            })
//            .transition(() ->
//                    !gamepads.exceedsThreshold(Analog.GP1_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD),
//                State.STANDBY
//            )
//            .onExit(() -> {
//                intake.forwardRegular();
//                intake.stop();
//                outtake.motor.setPower(0);
//            })
            .build();
    }

    @Override
    public void init_loop()
    {
//        boolean cameraIsReady = webcam.portal.getCameraState() == VisionPortal.CameraState.STREAMING;
//        telemetry.addData("Camera Is Ready?", cameraIsReady);
        telemetry.update();
    }

    @Override
    public void start()
    {
        fsm.start();
    }

    @Override
    public void run()
    {
        fsm.update();
        telemetry.addData("Current State", fsm.getState());
        telemetry.addData("Outtake Mode", outtake.getMode());
//        telemetry.addData("Goal Color", webcam.getGoalId() == AprilTagWebcam.RED_GOAL_ID ? "RED" : "BLUE");
        telemetry.addData("Outtake Motor Angular Velocity (ticks/sec)", ((DcMotorEx) outtake.motor.motor).getVelocity());
        telemetry.addData("Outtake Motor Angular Velocity (rev/min)", ((DcMotorEx) outtake.motor.motor).getVelocity() * 60 * (1 / Outtake.TICKS_PER_REVOLUTION));
    }
}
