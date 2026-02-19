package org.firstinspires.ftc.teamcode.teleop.tune;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(group="tune")
public class WebcamDistanceVelocityTuner extends BaseTeleOp {
    public static double targetAngularRate;
    public static boolean launch;

    public AprilTagWebcam webcam;
    public Intake intake;
    public Outtake outtake;

    @Override
    public void setup()
    {
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);
    }

    @Override
    public void run()
    {
        AprilTagDetection detection = webcam.getGoalDetection();

        if (detection == null)
        {
            telemetry.addData("Goal ID Is Detected", false);
            telemetry.addLine();
        }
        else
        {
            telemetry.addData("Goal ID Is Detected", true);
            telemetry.addLine();

            webcam.updateRange(detection.ftcPose.range);
            telemetry.addData("Range: ", webcam.getRange());
            telemetry.addLine();
        }

        outtake.setVelocity(targetAngularRate);

        telemetry.addData("Motor 1 Velocity", outtake.getLeftMotorVelocity());
        telemetry.addData("Motor 2 Velocity", outtake.getRightMotorVelocity());
        telemetry.addData("Average Motor Velocity", outtake.getAverageVelocity());

        if (launch)
            intake.forwardLaunch();
        else
            intake.stop();
    }

}
