package org.firstinspires.ftc.teamcode.teleop.comp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.control.Analog;
import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(group = "Comp")
public class FieldCentricComp extends BaseTeleOp
{
    public static final int WEBCAM_DEBOUNCE_TIME = 20; // milliseconds
    public static final int RUMBLE_DURATION = 500; // milliseconds

    public AbstractDrive drive;
    public Intake intake;
    public Outtake outtake;
    public AprilTagWebcam webcam;
    public ElapsedTime webcamTimer;

    @Override
    public void setup()
    {
        drive = new FieldCentricDrive(hardware);
        intake = new Intake(hardware);
        outtake = new Outtake(hardware);
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
        subsystems.addAll(List.of(drive, intake, outtake));
        webcamTimer = new ElapsedTime();
    }

    @Override
    public void init_loop()
    {
        if (gamepads.justPressed(Button.GP2_A)) webcam.toggleGoalId();
        AprilTagTelemetry.init_loop_telemetry(telemetry, webcam);
    }

    @Override
    public void start()
    {
        webcamTimer.reset();
    }

    @Override
    public void run()
    {
        telemetry.addData("Control Mode", outtake.getMode());

        if (outtake.getMode() == Outtake.ControlMode.WEBCAM_CONTROL)
        {
            if (webcamTimer.milliseconds() > WEBCAM_DEBOUNCE_TIME)
            {
                AprilTagDetection detection = webcam.getGoalDetection();
                if (gamepads.exceedsThreshold(Analog.GP2_RIGHT_TRIGGER, Outtake.TRIGGER_THRESHOLD))
                {
                    if (detection == null)
                    {
                        outtake.motor.setPower(0);
                        gamepad2.rumble(RUMBLE_DURATION);
                    }
                    else
                    {
                        AprilTagTelemetry.loop_telemetry(telemetry, detection);
                        double velocity = Outtake.calculateIdealFlywheelTangentialVelocity(detection.ftcPose.range);
                        double angularRate = Outtake.toAngularRate(velocity);
                        ((DcMotorEx) outtake.motor.motor).setVelocity(angularRate);
                    }
                }
                else
                {
                    outtake.motor.setPower(0);
                }
                webcamTimer.reset();
            }
        }
    }
}
