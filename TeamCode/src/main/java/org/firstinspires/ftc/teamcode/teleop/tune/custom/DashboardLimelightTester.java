package org.firstinspires.ftc.teamcode.teleop.tune.custom;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pinpointFiles.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

@Config
@TeleOp
public class DashboardLimelightTester extends BaseTeleOp {

    public Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;

    @Override
    public void setup()
    {
        limelight = hardwareMap.get(Limelight3A.class, "ll");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        pinpoint = hardware.pinpoint;
    }

    @Override
    public void run()
    {
        limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.DEGREES));

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            double limelightMountAngleDegrees = 13.0;

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightInches = 11.625;

            // distance from the target to the floor
            double goalHeightInches = 29.5;

            double angleToGoalDegrees = limelightMountAngleDegrees + ty;
            double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

            telemetry.addData("Target Distance", distanceFromLimelightToGoalInches);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}
