package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class ABigAutoFast extends ABigAuto {

    public void runOpMode() {
        cycleOutRamp = new DriveBase.Ramp(36, 12, 0.75, 0.25);
        cycleInRamp = new DriveBase.Ramp(36, 12, 0.75, 0.25);
        
        bluePaths[17] = new DriveBase.Path("white wall backdrop")
                .addPose(-54, 48, 180+30, 0.6)
                .addPose(-36, 58, 180, 0.6)
                .addPose(11, 58, 180, 0.7)
                .addPose(38, 49, 180-15, 0.6);
        bluePaths[18] = new DriveBase.Path("white wall floor")
                .addPose(-54, 48, 180+30, 0.6)
                .addPose(-36, 58, 180, 0.6)
                .addPose(11, 58, 180, 0.7)
                .addPose(20, 58, 180+0, 0.7);
        bluePaths[21] = new DriveBase.Path("white wall stack 1")
                .addPose(36, 48, 180, 0.6)
                .addPose(12, 58, 180, 0.6)
                .addPose(-36, 58, 180, 0.6)
                .addPose(-56, 42, 180+30, 0.7);
        bluePaths[22] = new DriveBase.Path("white wall stack 2", 1)
                .addPose(36, 48, 180, 0.6)
                .addPose(12, 58, 180, 0.6)
                .addPose(-36, 58, 180, 0.6)
                .addPose(-56, 42, 180+30, 0.7);
        
        super.runOpMode();
    }
    
    public void cycleIn(DriveBase.Path ipath, int scLevel) {
        ipath.reset();
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            bot.update();
            
            double dist;
            if (scLevel != 31) {
                dist = drive.followPath(ipath, cycleInRamp);
            } else {
                dist = drive.followPath(ipath);
            }
            if (dist < 0.5) break;
            if (dist < 24) { cycleExtend(scLevel); }
            drive.addTelemetry(telemetry);
            telemetry.update();
        }
        drive.stop();
        //bot.setMode(bot.mode.EXTEND);
    }
}