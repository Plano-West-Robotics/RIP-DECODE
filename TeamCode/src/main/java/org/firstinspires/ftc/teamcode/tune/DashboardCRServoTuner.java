package org.firstinspires.ftc.teamcode.tune;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

@TeleOp(group = "Tune")
public class DashboardCRServoTuner extends OpMode {
    @Override
    public void init() {
        FtcDashboard db = FtcDashboard.getInstance();

        for (CRServoImplEx s : hardwareMap.getAll(CRServoImplEx.class)) {
            s.setDirection(CRServo.Direction.FORWARD);
            String name = hardwareMap.getNamesOf(s).iterator().next();
            if (name == null) continue;

            db.addConfigVariable(this.getClass().getSimpleName(), name, new ValueProvider<String>() {
                final CRServoImplEx servo = s;
                double pow = 0;
                String verbatimInput = "";

                @Override
                public String get() {
                    return verbatimInput;
                }

                @Override
                public void set(String value) {
                    if (value.equals("")) {
                        this.servo.setPwmDisable();
                    } else {
                        try {
                            this.pow = Double.parseDouble(value);
                        } catch (NumberFormatException e) {
                            return;
                        }

                        this.servo.setPwmEnable();
                        this.servo.setPower(this.pow);
                    }

                    this.verbatimInput = value;
                }
            }, true);
        }
    }

    @Override
    public void loop() {}
}