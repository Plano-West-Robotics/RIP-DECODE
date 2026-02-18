package org.firstinspires.ftc.teamcode.teleop.tune;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(group="tune")
public class WebcamDistanceVelocityTuner extends BaseTeleOp {
    public static double targetAngularRate;
    public DcMotorEx motor1, motor2;

    public static final double F = 12;
    public static final double P = 44;
    public static final double I = 0.1;
    public static final double D = 0;

    public AprilTagWebcam webcam;

    @Override
    public void setup()
    {
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
        motor1 = hardwareMap.get(DcMotorEx.class, "o1");
        motor1.setVelocityPIDFCoefficients(P, I, D, F);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor2 = hardwareMap.get(DcMotorEx.class, "o2");
        motor2.setVelocityPIDFCoefficients(P, I, D, F);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void run()
    {
        AprilTagDetection detection = webcam.getGoalDetection();

        if (detection == null)
        {
            telemetry.addData("Goal ID not detected", false);
        }
        else
        {
            webcam.updateRange(detection.ftcPose.range);
            telemetry.addData("Range: ", webcam.getRange());
        }
        motor1.setVelocity(targetAngularRate);
        motor2.setVelocity(targetAngularRate);

    }

}
