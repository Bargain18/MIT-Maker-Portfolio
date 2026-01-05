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
public class TestBackdropAuto extends LinearOpMode {
    
    public double TIMEOUT_RETRIEVE_PIXELS = 25; // give up on pixels after this 26
    public double TIMEOUT_CYCLE_FULL = 23;   // don't attempt cycle after this
    
    public final double HUSKY_GAIN = -0.013/80;
    
    RobotHardware bot = new RobotHardware();
    NavCS nav = new NavCS();
    DualPad gpad = new DualPad();

    ElapsedTime waitTimer = new ElapsedTime();
    ElapsedTime autonTimer = new ElapsedTime();

    int propPos = 0;
    int alliance = 0;       // 1 == blue, 2 == red
    String[] allianceName = { ">>NOT SET<<", "blue", "RED" };

    Nav.Path redCycleOut = new Nav.Path()
                    .addPose(-11.25, 36, 2000, 12, 0.6)
                    .addPose(-11.25, 24, 2000, 12, 0.6)
                    .addPose(-11.25, 12, 2000)
                    .addPose(-11.25, -12, 2000)
                    .addPose(-11.25, -26.5, 2000)
                    .addPose(-11.25, -40, 2002 + 90.0/360.0, 6);
                    
    Nav.Path redCycleOut3 = new Nav.Path()
                    .addPose(-11.25, 36, 2000, 17, 0.6)
                    .addPose(-11.25, 24, 2000, 17, 0.6)
                    .addPose(-11.25, 12, 2000)
                    .addPose(-11.25, -12, 2000)
                    .addPose(-11.75, -36, 2002 + 90.0/360.0, 6); 
                    
    Nav.Path redCycleIn = new Nav.Path()
                    .addPose(-10.5, -48, 2001, 6, 0.70)
                    .addPose(-10.5, -36, 2001, 15) //LR changed to 8, might change back to default 
                    .addPose(-10.5, -24, 2001, 15)
                    .addPose(-11, 15, 0.217, 15)
                    .addPose(-21, 33.5, 0.22, 15);
    
    Nav.Path blueCycleOut = new Nav.Path()
                    .addPose(11.25, 36, 2000, 12, 0.6)
                    .addPose(11.25, 24, 2000, 12, 0.6)
                    .addPose(11.25, 12, 2000)
                    .addPose(11.25, -12, 2000)
                    .addPose(11.25, -26.5, 2000)
                    .addPose(11.75, -40, 2002 + 90.0/360.0, 6);

    Nav.Path blueCycleOut3 = new Nav.Path()
                    .addPose(11.25, 36, 2000, 12, 0.6)
                    .addPose(11.25, 24, 2000, 12, 0.6)
                    .addPose(11.25, 12, 2000)
                    .addPose(11.25, -12, 2000)
                    .addPose(11.25, -36, 2002 + 90.0/360.0, 6);
                    
    Nav.Path blueCycleIn = new Nav.Path()
                    .addPose(11, -48, 2001, 6, 0.65)
                    .addPose(11, -36, 2001, 15)
                    .addPose(11, -24, 2001, 15)
                    .addPose(11.5, 15, 0.283, 15)
                    .addPose(23.5, 34.5, 0.28, 15);

    double delay = 0.0;

    SimpleLogger logfile = null;

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        gpad.init();
        bot.launch(false);
        telemetry.addData("Status", "nav init");
        telemetry.update();
        nav.init(hardwareMap);
        nav.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // nav.setDriveVelocityPIDF(30.0, 0, 10.0, 13.0);
        telemetry.addData("Status", "camera init");
        telemetry.update();
        nav.initCamera(hardwareMap);
        // if (!isStopRequested() && !opModeIsActive()) {
        //     if (!gamepad1.left_bumper) {
        //         telemetry.addData("Status", "camera setExposure");
        //         telemetry.update();
        //         nav.setManualExposure(this, 20, 169);
        //     }
        // }
        while (!isStopRequested() && !opModeIsActive()){
            gpad.update(gamepad1, gamepad2);
            
            double ry = -gpad.left_stick_y -gpad.right_stick_y;
            double rx = gpad.right_stick_x;
            double rw = -gpad.left_stick_x;
            nav.updateTracking();
            nav.driveYXW(ry, rx, rw, 0.5);
            if (gpad.dpad_left && !gpad.previous.dpad_left) { setAuton(1,1); }  // blue left
            if (gpad.dpad_up && !gpad.previous.dpad_up)   { setAuton(1,2); }  // blue mid
            if (gpad.dpad_right && !gpad.previous.dpad_right) { setAuton(1,3); } // blue right
            if (gpad.dpad_down && !gpad.previous.dpad_down) { setAuton(1,0); }   // blue camera
            if (gpad.x) { setAuton(2,4); }        // RED left
            if (gpad.y) { setAuton(2,5); }        // RED mid
            if (gpad.b) { setAuton(2,6); }        // RED right
            if (gpad.a) { setAuton(2,0); }        // RED camera            

            // if (gpad.right_bumper) { bot.grab(3); }         // close scorer
            // if (gpad.shift.right_bumper) { bot.grab(0); }   // open scorer
            int delayinc = 0;
            if (gpad.right_bumper && !gpad.previous.right_bumper) delayinc = 1;
            if (gpad.shift.right_bumper && !gpad.previous.shift.right_bumper) delayinc = -1;
            delay = Range.clip(delay + delayinc, -30, 30);
            if (gpad.guide) { bot.setMode(bot.mode.SETUP); }
            nav.setFSpot(gpad.right_trigger >  0.5);

            bot.matrix.marqueeCycle("   7172   ", 0.03, 0, 0, 3-alliance, 0);
            bot.matrix.writeDisplay();
            bot.update();

            telemetry.addData("alliance", allianceName[alliance]);
            telemetry.addData(delay < 0 ? "wait before MATCH" : "wait after first INTAKE", delay);
            if (propPos != 0) telemetry.addData("Selected", propPos);
            else telemetry.addData("Selected", getRandomization());
            telemetry.addData("gamepad selection", propPos);
            telemetry.addData("camera prop alliance", nav.propProc.alliance);
            telemetry.addData("camera prop selection", getRandomization());
            telemetry.addData("HuskyLens Block", nav.getHusky());
            
            nav.addTelemetry(telemetry);
            bot.addTelemetry(telemetry);
            telemetry.addData("gpad", gpad.toString());
            telemetry.update();
        }
        
        if (isStopRequested()) return;
        autonTimer.reset();

        // driver pressed PLAY
        if (opModeIsActive()) {
            bot.grab(3);
            bot.launch(false);
            bot.spiv.setPosition(bot.SPIV_INIT);
            waitFor(0.1);
            
            // nav.resetIMUYaw();
            // sleep(15);
        }
        
        logfile = new SimpleLogger();
        logfile.open();
        logfile.add("Phase").add(bot.csv(true)).add(nav.csv(true)).headerLine();
        nav.logfile = logfile;
        
        
        if (propPos == 0) propPos = getRandomization();
        // make sure to do this after PLAY is pressed (resets heading offset)
        setAuton(alliance, propPos);
        
        if (opModeIsActive()) {
            if (delay < 0) waitFor(-delay);  

            // run the selected propPos
            switch (propPos) {
                case 4: run4(); break;
                case 5: run5(); break;
                case 6: run6(); break;
                case 1: run1(); break;
                case 2: run2(); break;
                case 3: run3(); break;
            }
        }

        logfile.close();
        
        // wait until the opmode ends
        while (opModeIsActive()) {
            nav.updateTracking();
            nav.updateAprilTagPosition();
            bot.update();
            nav.addTelemetry(telemetry);
            bot.addTelemetry(telemetry);
            telemetry.update();
        }
    }
    
    public int getRandomization() {
        // use camera selected path if none requested by driver
        return nav.getPropPos() + 1;
    }
    
    public void setAuton(int a, int p) {
        alliance = a;
        propPos = p;
        bot.setLEDPattern(alliance);
        nav.setAlliance(alliance, (alliance == 1 ? 0 : 3));
        if (alliance == 2) {
            nav.setHeading(-0.5);
            nav.setYX(-60, 15);
        }
        if (alliance == 1) {
            nav.setHeading(0);
            nav.setYX(60, 15);
        }
    }
    
    public void run1() {
        scorePurple(20, 47, 17, 0.030, 0.35);
        scoreYellow(41, 42, 0.5);
        
        cycleFull(blueCycleOut, 3, blueCycleIn, 15);
        cycleFull(blueCycleOut, 1, blueCycleIn, 15);
        cycleFull(blueCycleOut3, 3, blueCycleIn, 27, -0.07);
        park();
    }

    public void run2() {
        scorePurple(21, 44, 15, -0.06, 0.35);
        scoreYellow(36, 42, 0.5);
        cycleFull(blueCycleOut, 3, blueCycleIn, 15);
        cycleFull(blueCycleOut, 1, blueCycleIn, 15);
        cycleFull(blueCycleOut3, 3, blueCycleIn, 27, -0.07);
        park();
    }

    public void run3() {
        scorePurple(22, 49, 12.5, -0.11, 0.35);
        scoreYellow(29, 42, 0.5);
        cycleFull(blueCycleOut, 3, blueCycleIn, 15);
        cycleFull(blueCycleOut, 1, blueCycleIn, 15);
        cycleFull(blueCycleOut3, 3, blueCycleIn, 27, -0.07);
        park();
    }
    
    public void run4() {
        scorePurple(17, -46.5, 11.5, -0.4, 0.35);
        scoreYellow(-27.0, 42, 0.5);
        cycleFull(redCycleOut, 3, redCycleIn, 15);
        cycleFull(redCycleOut, 1, redCycleIn, 15);
        cycleFull(redCycleOut3, 3, redCycleIn, 27, 0.07);
        park();
    }

    public void run5() {
        scorePurple(18, -50, 15, -0.5, 0.35);
        scoreYellow(-33, 42, 0.5);
        cycleFull(redCycleOut, 3, redCycleIn, 15);
        cycleFull(redCycleOut, 1, redCycleIn, 15);
        cycleFull(redCycleOut3, 3, redCycleIn, 27, 0.07);
        park();
    }

    public void run6() {
        scorePurple(19, -48.5, 17, 0.455, 0.35);
        scoreYellow(-40.5, 42, 0.5);
        cycleFull(redCycleOut, 3, redCycleIn, 15);
        cycleFull(redCycleOut, 1, redCycleIn, 15);
        cycleFull(redCycleOut3, 3, redCycleIn, 27, 0.07);
        park();
    }
   
    public void scorePurple(int level, double ty, double tx, double th, double vel) {
        bot.setLevel(level);
        bot.setMode(bot.mode.EXTEND);
        gotoYXH(ty, tx, th, vel);
        waitTimer.reset();
        ElapsedTime purTimer = new ElapsedTime();
        while (opModeIsActive()) {
            double rx = 0;
            double ry = 0;
            if (waitTimer.seconds() > 3) break;
            nav.updateTracking();
            nav.updateAprilTagPosition();
            if (!nav.isHeading(th, 0.01)) purTimer.reset();
            if (purTimer.seconds() > 0.3) break;
            nav.driveYXH(ry, rx, th, 1);
            log("purple");
        }
        waitForIdle(2.5);
        bot.setMode(bot.mode.PURPLE);
        waitForIdle(5);
        bot.adjustTilt(1800);
        waitFor(0.1);
        bot.adjustTilt(0);
    }

    public void scoreYellow(double ty, double tx, double vel) {
        bot.setLevel(25);
        bot.setMode(bot.mode.EXTEND);
        nav.setRampProfile(20, 12, 1, 0.4);
        int tagsSeen = 0;
        while (opModeIsActive()) {
            nav.updateTracking();
            nav.updateAprilTagPosition();
            tagsSeen += nav.idCount;
            nav.moveRampDist(nav.distToYX(ty,tx));
            double distRemain = nav.moveToYXH(ty, tx, 2003.25, vel);
            if (distRemain < 3) break;
            bot.update();
            log("yellow");
        }
        waitTimer.reset();
        ElapsedTime navTimer = new ElapsedTime();
        while (opModeIsActive()) {
            double rx = 0;
            double ry = 0;
            if (waitTimer.seconds() > 3) break;
            nav.updateTracking();
            nav.updateAprilTagPosition();
            tagsSeen += nav.idCount;
            if (tagsSeen < 1) navTimer.reset();
            double errY = ty - nav.field.y;
            double errX = tx - nav.field.x;
            if (!nav.isHeading(0.25, 1)) navTimer.reset();
            if (Math.abs(errY) > 0.5) {
                navTimer.reset();
                rx = (errY > 0) ? 0.15 : -0.15;
            }
            if (errX > 0) {
                navTimer.reset();
                ry = -0.2;
            }
            if (navTimer.seconds() > 0.3) break;
            nav.driveYXH(ry, rx, 0.25, 1);
            log("yellow");
        }
        nav.driveStop(); 
        bot.setScoreSide(3);
        bot.setMode(bot.mode.SCORE);
        waitFor(0.5);
    }
    
    public void park() {
        bot.setLevel(16);
        bot.setMode(bot.mode.EXTEND);
    }
    
    public void gotoYXH(double y, double x, double h, double vel) {
        nav.setRampProfile(48, 24, 1, 0.25);
        while (opModeIsActive()) {
            nav.updateTracking();
            nav.updateAprilTagPosition();
            nav.moveRampDist(nav.distToYX(y,x));
            double distRemain = nav.moveToYXH(y, x, h, vel);
            if (distRemain < 1 && nav.isHeading(h, 0.005)) break;
            bot.update();
            log("gotoyxh");
        }
        nav.driveStop();
    }
    
    public void cycleOut(Nav.Path path, int inLevel) {
        nav.setPath(path);
        bot.intakeLevel = inLevel;
        // switch(inLevel){
        //     case 1: nav.setRampProfile(32, 12, 1, 0.25); break;
        //     case 3: nav.setRampProfile(32, 24, 1, 0.25); break;
        //     default: nav.setRampProfile(48, 24, 1, 0.25); break;
        // }
        nav.setRampProfile(24, 15, 1, 0.45);
        while (opModeIsActive()) {
            nav.updateTracking();
            //nav.updateAprilTagPosition();
            if(nav.field.x < -24 ) nav.setFSpot(true); 
            
            double d = nav.getPathRemainingDist();
            if (d < 2) break;
            //if (d < 36) { bot.setMode(bot.mode.INTAKE); }
            nav.moveRampDist(d); 
            nav.followPath();
            bot.update();
            log("cycleOut");
        }
        bot.setMode(bot.mode.INTAKE);
    }

    public void cycleIn(Nav.Path path, int scLevel) {
        nav.setPath(path);
        nav.setRampProfile(45, 24, 1, 0.45);
        while (opModeIsActive()) {
            nav.updateTracking();
            nav.updateAprilTagPosition();
            double d = nav.getPathRemainingDist();
            // if (d < 1) break;
            if (nav.field.x > 23) break;
            if (nav.field.x > 0) { 
                //bot.setLevel(scLevel); 
                cycleExtend(scLevel);
            }
            nav.moveRampDist(d); 
            nav.followPath();
            bot.update();
            log("cycleIn");
        }
        nav.driveStop();
        //bot.setMode(bot.mode.EXTEND);
    }

    public void cycleFull(Nav.Path opath, int inLevel, Nav.Path ipath, int scLevel) {
        cycleFull(opath,  inLevel, ipath,  scLevel, 0);
    }

    public void cycleFull(Nav.Path opath, int inLevel, Nav.Path ipath, int scLevel, double danceOffset) {
        if (autonTimer.seconds() > TIMEOUT_CYCLE_FULL) return; 
        cycleOut(opath, inLevel);
        retrievePixels(opath, danceOffset);
        
        if (inLevel == 3 && delay > 0)  waitFor( () -> autonTimer.seconds() > delay, delay);        
        cycleIn(ipath, scLevel);
        scoreWhite(scLevel, ipath.get(ipath.size()-1));
    }
    
    public void retrievePixels(Nav.Path opath, double danceOffset) {
        double huskyOffset = 0;
        //bot.setMode(bot.mode.INTAKE);
        double[] danceOrder = new double[] {0, 10/360.0, -10/360.0, 0, -20/360.0, 0, 20/360.0};
        //double[] danceOrder = new double[] {0, 15/360.0, -15/360.0};
        Nav.Pose endPose = opath.get(opath.size()-1);
        double tx = nav.field.x;
        double ty = endPose.y;
        double dist = nav.getUSDist();
        double wallx = nav.field.x - dist - 5.25;
        double wallxSum = wallx;
        double wallxCount = 1;
        boolean fwd = true;
        int danceStep = 0;
        ElapsedTime danceTimer = new ElapsedTime();
        while(opModeIsActive()) {
            if (autonTimer.seconds() >= TIMEOUT_RETRIEVE_PIXELS){
                bot.setMode(bot.mode.XFER);
                break; 
            }
            dist = nav.getUSDist();
            wallxSum += nav.field.x - dist - 5.25;
            wallxCount++;
            wallx = wallxSum / wallxCount;
            
            nav.updateTracking();
            nav.updateAprilTagPosition();
            //if (danceTimer.seconds() > 0.75) fwd = false;
            //if (dist < 2.75) fwd = false;
            if (nav.field.x < wallx + 7.75) fwd = false;
            if (dist < 6.5) fwd = false;

            //nav.setFSpot(fwd);
            if (fwd) {
                HuskyLens.Block block = nav.getHusky();
                huskyOffset = 0;
                if (block != null && bot.mTimer.seconds() > 0.4) {
                    huskyOffset = (block.left + block.width/2 - 160);
                }
                nav.driveYXH(0.4, huskyOffset * 0.31/80, 0.25 + danceOffset + danceOrder[danceStep], 0.5);
            } else if (nav.moveToYXH(ty, wallx+14.5, 0.25 + danceOffset, 0.4) < 1) {
                fwd = true;
                danceTimer.reset();
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
            logfile.add("retrievePixel").add(bot.csv())
                .add(nav.csv()).add(fwd).add(danceStep).add(wallx).add(dist)
                .add(huskyOffset).tsLine();
        }
        nav.setFSpot(false);
        nav.driveStop();
        bot.update();
        logfile.add("retrievePixel").add(bot.csv())
            .add(nav.csv()).add(fwd).add(danceStep).add(wallx).add(dist).tsLine();
    }
    
    public void scoreWhite(int level, Nav.Pose pose) {
        waitFor(() -> cycleExtend(level), 3);
        // waitFor(0.15);
        // nav.driveYXW(-0.2, 0, 0, 1.0);
        // waitFor(()->((nav.field.x > 35.5) || (bot.getBallSw() == 3)), 3);
        waitTimer.reset();
        ElapsedTime navTimer = new ElapsedTime();
        double ty = pose.y;
        double tx = pose.x;
        while (opModeIsActive()) {
            double rx = 0;
            double ry = 0;
            if (waitTimer.seconds() > 3) break;
            nav.updateTracking();
            nav.updateAprilTagPosition();
            // tagsSeen += nav.idCount;
            // if (tagsSeen < 1) navTimer.reset();
            double errY = ty - nav.field.y;
            double errX = tx - nav.field.x;
            if (!nav.isHeading(pose.h, 0.05)) navTimer.reset();
            //if (!bot.isMode(bot.mode.EXTENDED)) navTimer.reset();
            if (bot.getBallSw() == 3) break;
            if (Math.abs(errY) > 1) {
                navTimer.reset();
                rx = (errY > 0) ? 0.15 : -0.15;
            }
            if (errX > 0) {
                navTimer.reset();
                ry = -0.2;
            }
            if (navTimer.seconds() > 0.3 && bot.isMode(bot.mode.EXTENDED)) break;
            nav.driveYXH(ry, rx, pose.h, 1);
            bot.update();
            
            log("white");
        }
        nav.driveStop();
        nav.driveStop(); 
        bot.setScoreSide(3);
        bot.setMode(bot.mode.SCORE);
        // waitFor( () -> !bot.isMode(bot.mode.SCORE) , 3);
        waitFor(0.5);
    }
    
    public boolean cycleExtend(int level) {
        if (bot.isMode(bot.mode.EXTEND) || bot.isMode(bot.mode.EXTENDED)) return true;
        if (bot.isMode(bot.mode.WAIT) || bot.isMode(bot.mode.SWING)) {bot.setLevel(level); bot.setMode(bot.mode.EXTEND); return false;}
        return false;
    }
    
    
    public boolean waitFor(double timeout) {
        return waitFor( () -> false, timeout);
    }
    public boolean waitFor(Supplier<Boolean> c, double timeout) {
        waitTimer.reset();
        while (opModeIsActive()) {
            nav.updateTracking();
            nav.updateAprilTagPosition();
            if (c.get()) return true;
            if (waitTimer.seconds() > timeout) return false;
            bot.update();
            log("waitFor");
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
        logfile.add(phase).add(bot.csv()).add(nav.csv()).tsLine();
    }

}
