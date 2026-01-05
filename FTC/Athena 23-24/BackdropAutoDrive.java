package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

@Autonomous
public class BackdropAutoDrive extends LinearOpMode {
    
    public double TIMEOUT_RETRIEVE_PIXELS = 27; // give up on pixels after this
    public double TIMEOUT_CYCLE_FULL = 25;   // don't attempt cycle after this
    
    DriveBase.Path wallPathRedOut = new DriveBase.Path()
                    .addPose(24, -56, 180, 0.4)
                    .addPose(-38, -57, 180, 0.5)
                    .addPose(-58, -40, 160, 0.3) //170 -- 160
                    .addPose(-58, -40, 160, 0.2);
    DriveBase.Path wallPathRedIn = new DriveBase.Path()
                    .addPose(-38, -56, 180, 0.5)
                    .addPose(-7, -57, 180, 0.55)
                    .addPose(20, -57, 180, 0.5)
                    .addPose(27, -52, 190, 0.4)
                    .addPose(34, -47, 190, 0.4); //43
                    
    
    RobotHardware bot = new RobotHardware();
    DualPad gpad = new DualPad();
    DriveBaseCS drive = new DriveBaseCS();
    
    ElapsedTime waitTimer = new ElapsedTime();
    ElapsedTime autonTimer = new ElapsedTime();
    
    double purpleXOffset = 0;
    double purpleYOffset = 0;
    
    DriveBase.Ramp purpleRamp = new DriveBase.Ramp(12, 3, 0.35, 0.23);
    DriveBase.Ramp yellowRamp = new DriveBase.Ramp(20, 12, 1, 0.2);
    DriveBase.Ramp wallRamp = new DriveBase.Ramp(20, 6, 0.75, 0.1);
    DriveBase.Ramp whiteRamp = new DriveBase.Ramp(20, 6, 0.75, 0.1);
    
    boolean distFlag = true;
    SimpleLogger logfile = null;

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        gpad.init();
        drive.init(hardwareMap);
        drive.initCamera(hardwareMap);
        drive.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPosition(16.5, -62.5, -90);
        
        logfile = new SimpleLogger();
        logfile.open();
        logfile.add("Phase").add(bot.csv(true)).add(drive.csv(true)).headerLine();
        drive.logfile = logfile;
        
        while (!isStopRequested() && !opModeIsActive()) {
            gpad.update(gamepad1, gamepad2);
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);

            double rx = -gpad.left_stick_y -gpad.right_stick_y;
            double ry = -gpad.right_stick_x;
            double rw = -gpad.left_stick_x;
            drive.driveXYW(rx, ry, rw, 0.5);
            
            if (gpad.guide) { bot.setMode(bot.mode.SETUP); }
            if (gpad.dpad_left && !gpad.previous.dpad_left) {purpleXOffset -= 0.1;}
            if (gpad.dpad_right && !gpad.previous.dpad_right) {purpleXOffset += 0.1;}
            if (gpad.dpad_up && !gpad.previous.dpad_up) {purpleYOffset += 0.1;}
            if (gpad.dpad_down && !gpad.previous.dpad_down) {purpleYOffset -= 0.1;}
            
            bot.update();
            telemetry.addData("purplexOffset", purpleXOffset);
            telemetry.addData("purpleyOffset", purpleYOffset);
            drive.addTelemetry(telemetry);
            telemetry.update();
        }

        // driver pressed PLAY
        if (opModeIsActive()) {
            bot.grab(3);
            bot.launch(false);
            bot.spiv.setPosition(bot.SPIV_INIT);
            waitFor(0.1);
        }
        
        run4();
        
        // wait until the opmode ends
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            bot.update();
            drive.addTelemetry(telemetry);
            bot.addTelemetry(telemetry);
            telemetry.update();
        }
    }
    
    public void run4() {
        scorePurple(12.0, -49.5, -60, 0.5, 17);
        scoreYellow(14, 39, -28.5);
        bot.intakeLevel = 3;
        cycleOut(wallPathRedOut, wallRamp);
        retrievePixels(wallPathRedOut);
        // waitFor(3);
        cycleIn(wallPathRedIn, wallRamp, 15);
        scoreWhite(15, wallPathRedIn.last());
        bot.intakeLevel = 1;
        cycleOut(wallPathRedOut, wallRamp);
        retrievePixels(wallPathRedOut);
        // waitFor(3);
        cycleIn(wallPathRedIn, wallRamp, 15);
        scoreWhite(15, wallPathRedIn.last());
    }
    
    public void run5() {
        scorePurple(16.5, -50, -90, 0.5, 17);
        scoreYellow(14, 35, -41);
        bot.intakeLevel = 3;
        cycleOut(wallPathRedOut, wallRamp);
        retrievePixels(wallPathRedOut);
        // waitFor(3);
        cycleIn(wallPathRedIn, wallRamp, 15);
        scoreWhite(15, wallPathRedIn.last());
        cycleOut(wallPathRedOut, wallRamp);
        retrievePixels(wallPathRedOut);
        // waitFor(3);
        cycleIn(wallPathRedIn, wallRamp, 15);
        scoreWhite(15, wallPathRedIn.last());
    }
   

    public void scorePurple(double tx, double ty, double th, double vel, int scLevel) {
        bot.setLevel(scLevel);
        bot.setMode(bot.mode.EXTEND);
        moveTo(tx, ty, th, vel, purpleRamp);
        
        ElapsedTime purTimer = new ElapsedTime();
        
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            if (!drive.isHeading(th, 5)) purTimer.reset();
            if (purTimer.seconds() > 0.3) break;
            drive.driveXYW(0, 0, drive.calcRVW(th), 1);
        }
        
        waitForIdle(2.5);
        bot.setMode(bot.mode.PURPLE);
        waitForIdle(5);
        bot.adjustTilt(1800);
        waitFor(0.1);
        bot.adjustTilt(0);
    }
    
    public void scoreYellow(int scLevel, double tx, double ty) {
        double vel = 0.5;
        bot.setLevel(scLevel);
        bot.setMode(bot.mode.EXTEND);
        moveTo(tx, ty, 180.0, vel, yellowRamp);
        int tagsSeen = 0;
        waitTimer.reset();
        ElapsedTime navTimer = new ElapsedTime();
        while (opModeIsActive()) {
            double rx = 0;
            double ry = 0;
            if (waitTimer.seconds() > 10) break;
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            tagsSeen += drive.idCount;
            if (tagsSeen < 1) navTimer.reset();
            double errX = tx - drive.field.x;
            double errY = ty - drive.field.y;
            if (!drive.isHeading(180, 3)) navTimer.reset();
            if (Math.abs(errY) > 0.5) {
                navTimer.reset();
                ry = (errY > 0) ? -0.2 : 0.2;
            }
            if (errX > 0) {
                navTimer.reset();
                rx = -0.2;
            }
            if (navTimer.seconds() > 0.3) break;
            drive.driveXYW(rx, ry, drive.calcRVW(180), 1);
            log("yellow");
            telemetry.addData("target", "tx=%.2f ty=%.2f", tx, ty);
            drive.addTelemetry(telemetry);
            telemetry.update();
        }
        drive.stop(); 
        bot.setScoreSide(3);
        bot.setMode(bot.mode.SCORE);
        waitFor(0.5);
    }

    // public void scoreYellow(double tx, double ty, double vel, double tag) {
    //     scoreYellow(tx, ty, vel, 25, tag);
    // }

    // public void scoreYellow(double tx, double ty, double vel, int scLevel, double tag) {
    //     bot.setLevel(scLevel);
    //     bot.setMode(bot.mode.EXTEND);
    //     moveTo(tx, ty, 180.0, vel, yellowRamp);
        
    //     waitTimer.reset();
    //     ElapsedTime navTimer = new ElapsedTime();
    //     double range = 13;
    //     double bearing = 0;
    //     while (opModeIsActive()) {
    //         drive.updateOTOS();
    //         drive.updateAprilTagPosition(telemetry);
    //         bot.update();
            
    //         double rx = 0;
    //         double ry = 0;
    //         if (waitTimer.seconds() > 3) break;
    //         double[] tagValues = drive.getTagValues(tag);
    //         if (tagValues == null) {
    //             telemetry.addData("tagSeen", false);
    //             telemetry.update();
    //             continue;
    //         }
    //         double rError = tagValues[0] - range;
    //         double bError = bearing - tagValues[1];
    //         if (!drive.isHeading(180, 5)) navTimer.reset();
    //         if (rError > 0.5) {
    //             navTimer.reset();
    //             rx = -0.2;
    //         }
    //         if (Math.abs(bError) > 10) {
    //             navTimer.reset();
    //             ry = (bError > 0) ? 0.15 : -0.15;
    //         }
    //         if (navTimer.seconds() > 0.2) break;
    //         drive.driveXYW(rx, ry, drive.calcRVW(180), 1);
    //         log("yellow");
            
    //         telemetry.addData("tagSeen", true);
    //         telemetry.addLine(String.format("rError: %.2f  bError: %.2f", rError, bError));
    //         telemetry.addLine(String.format("range: %.2f bearing: %.2f", tagValues[0], tagValues[1]));
    //         telemetry.addLine(String.format("rx: %.2f ry: %.2f", rx, ry));
    //         telemetry.update();
    //     }
    //     drive.stop();
    //     waitForIdle(2.5);
    //     bot.setScoreSide(3);
    //     bot.setMode(bot.mode.SCORE);
    //     waitFor(0.5);
    // }
    
    public void retrievePixels(DriveBase.Path opath) {
        double huskyOffset = 0;
        bot.setMode(bot.mode.INTAKE);
        double[] danceOrder = new double[] {0, -10, -15, -15, -15, -15};
        //double[] danceOrder = new double[] {0, 15/360.0, -15/360.0};
        DriveBase.Pose endPose = opath.get(opath.size()-1);
        double tx = endPose.x;
        double ty = endPose.y;
        double danceAngle = endPose.h;
        double dist = drive.getUSDist();
        double wallx = drive.field.x - dist - 5.25;
        double wallxSum = wallx;
        double wallxCount = 1;
        boolean fwd = true;
        int danceStep = 0;
        
        double fwdVel = 0.25;
        double bwdVel = 0.3;
        drive.setFSpot(true);
        while(opModeIsActive()) {
            // if (autonTimer.seconds() >= TIMEOUT_RETRIEVE_PIXELS){
            //     bot.setMode(bot.mode.XFER);
            //     break; 
            // }
            dist = drive.getUSDist();
            wallxSum += drive.field.x - dist - 5.25;
            wallxCount++;
            wallx = wallxSum / wallxCount;
            
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            //if (danceTimer.seconds() > 0.75) fwd = false;
            //if (dist < 2.75) fwd = false;
            if(distFlag){
                if (drive.field.x < wallx + 3.4) fwd = false;
                if (dist < 3.4) fwd = false;
            } else {
                if(drive.field.x < -72 + 7.75) {fwd = false;}
            }

            //nav.setFSpot(fwd);
            if (fwd) {
                HuskyLens.Block block = drive.getHusky();
                huskyOffset = 0;
                if (block != null && bot.mTimer.seconds() > 0.4) {
                    huskyOffset = (block.left + block.width/2 - 160);
                }
                double heading = drive.calcRVW(danceAngle + danceOrder[danceStep]);
                drive.driveXYW(fwdVel, huskyOffset * 0.31/80, heading, 1);
            } else if (drive.moveToXYH(tx, ty, danceAngle + danceOrder[danceStep], bwdVel) < 0.5) {
                //(inside)-0.4 -> 0.5, outside-0.5 -> 0.5
                fwd = true;
                if (bot.intakeLevel <= 1) {
                    danceStep++;
                    if (danceStep >= danceOrder.length) {
                        bot.setMode(bot.mode.XFER);
                        break;
                    }
                } else { 
                     bot.intakeLevel = 1;
                     bot.resetMode(bot.mode.INTAKE);
                }
            }
            bot.update(); 
            if(!bot.isMode(bot.mode.INTAKE)) break;
            drive.addTelemetry(telemetry);
            telemetry.addData("fwd", fwd);
            telemetry.addData("back pose", "x=%.2f y=%.2f h=%.2f", tx, ty, danceAngle);
            telemetry.addData("usdist", dist);
            telemetry.addData("danceStep", danceStep);
            telemetry.update();
            
            logfile.add("retrievePixel").add(bot.csv())
                .add(drive.csv()).add(fwd).add(danceStep).add(wallx).add(dist)
                .add(huskyOffset).tsLine();
        }
        drive.setFSpot(false);
        drive.stop();
        logfile.add("retrievePixel").add(bot.csv())
                .add(drive.csv()).add(fwd).add(danceStep).add(wallx).add(dist)
                .add(huskyOffset).tsLine();
    }
    
    public void cycleIn(DriveBase.Path path, DriveBase.Ramp ramp, int scLevel) {
        path.reset();
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            bot.update();
            
            double dist = drive.followPath(path);
            if (drive.field.x > 23) break;
            if (drive.field.x > 0) { 
                cycleExtend(scLevel);
            }
            drive.addTelemetry(telemetry);
            telemetry.update();
        }
        drive.stop();
        //bot.setMode(bot.mode.EXTEND);
    }
    
    public boolean cycleExtend(int level) {
        if (bot.isMode(bot.mode.EXTEND) || bot.isMode(bot.mode.EXTENDED)) return true;
        if (bot.isMode(bot.mode.WAIT) || bot.isMode(bot.mode.SWING)) {bot.setLevel(level); bot.setMode(bot.mode.EXTEND); return false;}
        return false;
    }
    
    public void scoreWhite(int level, DriveBase.Pose pose) {
        waitFor(() -> cycleExtend(level), 3);
        waitTimer.reset();
        ElapsedTime navTimer = new ElapsedTime();
        double tx = pose.x;
        double ty = pose.y;
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            bot.update();
            
            double rx = 0;
            double ry = 0;
            // if (waitTimer.seconds() > 3) break;
            double errX = tx - drive.field.x;
            double errY = ty - drive.field.y;
            if (!drive.isHeading(pose.h, 10)) navTimer.reset();
            //if (!bot.isMode(bot.mode.EXTENDED)) navTimer.reset();
            if (bot.getBallSw() == 3) break;
            if (Math.abs(errY) > 1) {
                navTimer.reset();
                ry = (errY > 0) ? -0.15 : 0.15;
            }
            if (errX > 0) {
                navTimer.reset();
                rx = -0.2;
            }
            if (navTimer.seconds() > 0.3 && bot.isMode(bot.mode.EXTENDED)) break;
            drive.driveXYW(rx, ry, drive.calcRVW(pose.h), 1);
            drive.addTelemetry(telemetry);
            telemetry.addData("target pos", "x=%.2f y=%.2f h=%.2f", tx, ty, pose.h);
            telemetry.update();
        }
        drive.stop();
        bot.setScoreSide(3);
        bot.setMode(bot.mode.SCORE);
        // waitFor( () -> !bot.isMode(bot.mode.SCORE) , 3);
        waitFor(0.5);
    }
    
    public void cycleOut(DriveBase.Path path, DriveBase.Ramp ramp) {
        path.reset();
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            bot.update();
            
            double dist = drive.followPath(path);
            if (dist < 1.5) break;
            if (drive.field.x < -32) bot.setMode(bot.mode.INTAKE);
            
            drive.addTelemetry(telemetry);
            telemetry.addData("current path pose", path.get(path.current).toString());
            telemetry.update();
        }
        drive.stop();
    }
        
    
    public void moveTo(double tx, double ty, double th, double vel, DriveBase.Ramp ramp) {
        while (opModeIsActive()) {
            drive.updateOTOS();
            bot.update();
            
            double dist = drive.moveToXYHRamp(tx, ty, th, vel, ramp);
            if (dist < 1) break;
            
            drive.addTelemetry(telemetry);
            telemetry.update();
        }
        drive.stop();
    }
    
    public boolean waitFor(double timeout) {
        return waitFor( () -> false, timeout);
    }
    public boolean waitFor(Supplier<Boolean> c, double timeout) {
        waitTimer.reset();
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            if (c.get()) return true;
            if (waitTimer.seconds() > timeout) return false;
            bot.update();
            // nav.addTelemetry(telemetry);
            bot.addTelemetry(telemetry);
            telemetry.update();
        }
        return false;
    }
    public boolean waitForIdle(double timeout) {
        return waitFor( () -> !bot.isBusy(), timeout); 
    }
    
    public void log(String phase) {
        if (logfile == null) return;
        logfile.add(phase).add(bot.csv()).add(drive.csv()).tsLine();
    }
}
