package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@TeleOp(group = "Test")
public class OuttakeMaxVelocityTest extends LinearOpMode
{
    public DcMotorEx motor;

    double currentTPS;
    double maxTPS = 0;

    double currentRPM;
    double maxRPM = 0;

    @Override
    public void runOpMode()
    {
        motor = hardwareMap.get(DcMotorEx.class, "o");

        waitForStart();
        
        motor.setPower(1);

        while (opModeIsActive())
        {
            currentTPS = motor.getVelocity();
            if (currentTPS > maxTPS) maxTPS = currentTPS;

            currentRPM = (currentTPS / Outtake.TICKS_PER_REVOLUTION) * 60.0;
            if (currentRPM > maxRPM) maxRPM = currentRPM;

            telemetry.addData("Current TPS", "%.2f", currentTPS);
            telemetry.addData("Current RPM", "%.2f", currentRPM);
            telemetry.addData("Max TPS", "%.2f", maxTPS);
            telemetry.addData("Max RPM", "%.2f", maxRPM);

            telemetry.update();
        }
    }
}