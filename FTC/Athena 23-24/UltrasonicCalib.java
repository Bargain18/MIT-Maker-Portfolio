
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class UltrasonicCalib extends LinearOpMode {

    public AnalogInput usdist = null;

    @Override
    public void runOpMode() {
        usdist = hardwareMap.get(AnalogInput.class, "usdist");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double distV = usdist.getVoltage();
            double maxV = usdist.getMaxVoltage();
            double dist = Range.scale(distV, 0.15, 0.207, 5.5, 9.25);
            telemetry.addData("Status", "Running");
            telemetry.addData("distV", "%.3f", distV);
            telemetry.addData("maxV", "%.3f", maxV);
            telemetry.addData("dist", "%.2f", dist);
            telemetry.update();

        }
    }
}
