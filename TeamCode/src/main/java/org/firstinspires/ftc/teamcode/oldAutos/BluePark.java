package org.firstinspires.ftc.teamcode.oldAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.RobotCentricDrive;

@Autonomous(group = "Comp")
public class BluePark extends LinearOpMode
{
    public Hardware hardware;
    public RobotCentricDrive drive;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new Hardware(hardwareMap);
        drive = new RobotCentricDrive(hardware);

        waitForStart();

        if (isStarted()) {
            drive.drivetrainMotors.setPower(0.2, 0.2, 0.2, 0.2);
            sleep(2900);

            drive.drivetrainMotors.setPower(0, 0, 0, 0);
            sleep(1000);

            drive.drivetrainMotors.setPower(-0.25, 0.25, 0.25, -0.25);
            sleep(5500);

            drive.drivetrainMotors.setPower(0, 0, 0, 0);
        }
    }
}
