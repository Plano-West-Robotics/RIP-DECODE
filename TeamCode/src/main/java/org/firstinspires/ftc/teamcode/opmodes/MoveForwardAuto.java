package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.RobotCentricDrive;

@Autonomous(group = "Comp")
public class MoveForwardAuto extends LinearOpMode {
    public Hardware hardware;
    public AbstractDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(hardwareMap);
        drive = new RobotCentricDrive(hardware);

        waitForStart();

        if (isStarted())
        {
            drive.drivetrainMotors.setPower(0.2, 0.2, 0.2, 0.2);
            sleep(3000);
            drive.drivetrainMotors.setPower(0, 0, 0, 0);
        }
    }
}
