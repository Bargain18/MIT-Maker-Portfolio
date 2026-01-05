package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

@TeleOp

public class NavTuner extends LinearOpMode {

    Nav nav = null;
    DualPad gpad = null;
    boolean useIMU = false;
    
    double tuneDist = 72;

    @Override
    public void runOpMode() {
        gpad = new DualPad();
        gpad.init();
        nav = new Nav();
        nav.init(hardwareMap);
        nav.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
             hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        
        waitForStart();

        while (opModeIsActive()) {
            nav.updateTracking(useIMU);
            gpad.update(gamepad1, gamepad2);
            double ry = -gpad.left_stick_y - gpad.right_stick_y;
            double rx = gpad.right_stick_x;
            double rw = -gpad.left_stick_x;

            if (gpad.shift.a && !gpad.previous.shift.a) straightTune();
            if (gpad.shift.x && !gpad.previous.shift.x) trackWidthTune();
            if (gpad.shift.y && !gpad.previous.shift.y) strafeTune();
            if (gpad.dpad_up) tuneDist = 72;
            if (gpad.dpad_down) tuneDist = 48;
            
            nav.driveYXW(ry, rx, rw, 0.5);
            telemetry.addData("useIMU", useIMU);
            telemetry.addData("tuneDist", tuneDist);
            nav.addTelemetry(telemetry);
            telemetry.update();
        }
    }
    
    public void straightTune() {
        nav.setYX(0,0);
        nav.setHeading(0);
        nav.setRampProfile(48, 0, 1, 0.15);
        while (opModeIsActive()) {
            nav.updateTracking(useIMU);
            nav.moveRampDist(nav.distToYX(72,0));
            if (nav.moveToYXH(72,0,0,0.6)<0.5) break;
        }
        nav.driveStop();
    }
    
    public void trackWidthTune() {
        useIMU = false;
        nav.setYX(0,0);
        nav.setHeading(0);
        nav.driveYXW(0,0,0.3,1);
        while (opModeIsActive()) {
            nav.updateTracking(useIMU);
            if (Math.abs(nav.normalizeAngle(nav.getHeading()-0.5)) < 1.0/360) break;
        }
        nav.driveStop();
    }
    
    public void strafeTune() {
        useIMU = true;
        nav.setYX(0,0);
        nav.setHeading(0);
        nav.setRampProfile(48, 0, 1, 0.15);
        while (opModeIsActive()) {
            nav.updateTracking(useIMU);
            nav.moveRampDist(nav.distToYX(0,tuneDist));
            if (nav.moveToYXH(0,tuneDist,0,0.6)<0.5) break;
        }
        nav.driveStop();
    }
}
