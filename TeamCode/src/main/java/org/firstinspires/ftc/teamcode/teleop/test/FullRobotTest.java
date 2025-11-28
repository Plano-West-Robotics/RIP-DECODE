package org.firstinspires.ftc.teamcode.teleop.test;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.control.Analog;
import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.teamcode.teleop.tune.DashboardWebcamAngularPIDFTuner;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class FullRobotTest extends BaseTeleOp
{
    /*
    The gamepad will rumble once the flywheel's angular velocity deviates this amount or less from
    flywheelTargetAngularVelocity.
     */
    public static final double FLYWHEEL_VELOCITY_ERROR_TOLERANCE = 50;

    public AprilTagWebcam webcam;
    public AbstractDrive drive;
    public PIDFController controller;

    public Intake intake;
    public Outtake outtake;
    public OuttakeState outtakeState;

    public double intakeMotorPower = Intake.REGULAR_POWER;
    public boolean intakeIsSpinning;
    public double flywheelTargetAngularVelocity; // ticks per second

    public enum OuttakeState
    {
        INIT,
        ACCELERATING,
        // AT_SPEED,
        LAUNCHING
    }

    @Override
    public void setup()
    {
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
        drive = new FieldCentricDrive(hardware);
        controller = new PIDFController(
            DashboardWebcamAngularPIDFTuner.P,
            DashboardWebcamAngularPIDFTuner.I,
            DashboardWebcamAngularPIDFTuner.D,
            DashboardWebcamAngularPIDFTuner.F
        );

        intake = new Intake(hardware);
        outtake = new Outtake(hardware);
        outtakeState = OuttakeState.INIT;

        intakeMotorPower = Intake.REGULAR_POWER;
        intakeIsSpinning = false;
    }

    @Override
    public void run()
    {
        telemetry.addData("Outtake Mode", outtake.getMode());

        switch (outtakeState)
        {
            case INIT:
                if (gamepads.justPressed(Button.GP1_A)) intakeIsSpinning = !intakeIsSpinning;
                if (gamepads.justPressed(Button.GP1_B))
                {
                    intakeMotorPower *= -1;
                }
                if (gamepads.justPressed(Button.GP1_X))
                {
                    if (Math.abs(intakeMotorPower) == Intake.LAUNCH_POWER)
                    {
                        intakeMotorPower = Math.signum(intakeMotorPower) * Intake.REGULAR_POWER;
                    }
                    else if (Math.abs(intakeMotorPower) == Intake.REGULAR_POWER)
                    {
                        intakeMotorPower = Math.signum(intakeMotorPower) * Intake.LAUNCH_POWER;
                    }
                }

                if (gamepads.exceedsThreshold(Analog.GP2_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD))
                {
                    intakeMotorPower = Intake.LAUNCH_POWER;
                    intake.motor.setPower(intakeMotorPower);

                    if (outtake.getMode() == Outtake.ControlMode.MANUAL_CONTROL)
                    {
                        // TODO: Change to DcMotorEx's setVelocity()
                        outtake.motor.noEncoder();
                        outtake.motor.setPower(Outtake.POWER);
                        flywheelTargetAngularVelocity = 2000;
                        outtakeState = OuttakeState.ACCELERATING;
                    }
                    else if (outtake.getMode() == Outtake.ControlMode.WEBCAM_CONTROL)
                    {
                        AprilTagDetection detection = webcam.getGoalDetection();
                        if (detection == null)
                        {
                            outtake.setMode(Outtake.ControlMode.MANUAL_CONTROL);
                            break;
                        }
                        else
                        {
                            telemetry.addData("Range", detection.ftcPose.range);
                            telemetry.addData("Bearing", detection.ftcPose.bearing);

                            double rx = controller.calculate(detection.ftcPose.bearing, 0);
                            drive.drive(0, 0, rx);

                            telemetry.addData("RX", rx);

                            flywheelTargetAngularVelocity = Outtake.toAngularRate(Outtake.calculateIdealFlywheelTangentialVelocity(detection.ftcPose.range));
                            ((DcMotorEx) outtake.motor).setVelocity(flywheelTargetAngularVelocity);

                            outtakeState = OuttakeState.ACCELERATING;
                        }
                    }
                }
                break;
            case ACCELERATING:
                outtake.stoppers.stop();
                if (!gamepads.exceedsThreshold(Analog.GP2_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD))
                {
                    outtake.motor.setPower(0);
                    outtakeState = OuttakeState.INIT;
                }
                if (Math.abs(flywheelTargetAngularVelocity - ((DcMotorEx) outtake.motor).getVelocity()) < FLYWHEEL_VELOCITY_ERROR_TOLERANCE)
                {
                    gamepad1.rumble(20);
                    gamepad2.rumble(20);

                    if (gamepads.isPressed(Button.GP2_A))
                    {
                        outtake.stoppers.standby();
                        outtakeState = OuttakeState.LAUNCHING;
                    }
                }
                break;
//            case AT_SPEED:
//                break;
            case LAUNCHING:
                if (Math.abs(flywheelTargetAngularVelocity - ((DcMotorEx) outtake.motor).getVelocity()) > FLYWHEEL_VELOCITY_ERROR_TOLERANCE)
                {
                    outtakeState = OuttakeState.ACCELERATING;
                }
                break;
        }
    }
}
