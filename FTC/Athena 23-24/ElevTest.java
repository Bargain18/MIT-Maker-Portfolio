package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class ElevTest extends LinearOpMode {

    private DcMotorEx elev;
    private DigitalChannel elevLimit; 

    @Override
    public void runOpMode() {
        
        elev = hardwareMap.get(DcMotorEx.class, "elev");
        elev.setDirection(DcMotorSimple.Direction.REVERSE);
        elevLimit = hardwareMap.get(DigitalChannel.class, "elim");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        boolean aLast = false;
        boolean retracting = false;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            
            if (retracting && timer.milliseconds() > 475) {
                elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                retracting = false;
            }            
            
            boolean aThis = gamepad1.a;
            if (aThis && !aLast) {
                elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elev.setVelocity(-1000);
                retracting = true;
                timer.reset();
            }
            aLast = aThis;

            telemetry.addData("elev.pos", elev.getCurrentPosition());
            telemetry.addData("elev.mode", "%s", elev.getMode());
            telemetry.addData("elev.current", "%.2fA", elev.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("elevLimit.getState", elevLimit.getState());
            telemetry.addData("retracting", retracting);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
