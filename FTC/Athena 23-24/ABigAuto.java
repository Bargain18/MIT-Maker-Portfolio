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
public class ABigAuto extends LinearOpMode {
    
    public double TIMEOUT_RETRIEVE_PIXELS = 25.5; // give up on pixels after this
    public double TIMEOUT_CYCLE_FULL = 24;   // don't attempt cycle after this

    RobotHardware bot = new RobotHardware();
    DualPad gpad = new DualPad();
    DriveBaseCS drive = new DriveBaseCS();
    
    ElapsedTime waitTimer = new ElapsedTime();
    ElapsedTime autonTimer = new ElapsedTime();
    
    DriveBase.Ramp purpleRamp = new DriveBase.Ramp(12, 3, 0.35, 0.23);
    DriveBase.Ramp yellowRamp = new DriveBase.Ramp(36, 12, 1, 0.2);
    DriveBase.Ramp cycleOutRamp = new DriveBase.Ramp(36, 12, 0.75, 0.15);
    DriveBase.Ramp cycleInRamp = new DriveBase.Ramp(36, 12, 0.75, 0.15);
    
    boolean distFlag = true;
    SimpleLogger logfile = null;
    
    int alliance = 0;
    String[] allianceLabels = { ">>not set<<", "blue", "RED" };
    int startPosition = 0;
    String[] startLabels = { "backdrop", "audience (door)", "audience (wall)" };
    String[] propLabels = { "LEFT", "CENTER", "RIGHT", "CAMERA" };
    int propPosition = 3; // requested prop randomization
    int randomization = 0;
    int purplePath = 0;
    int yellowPath = 0;
    int cycle1Path = 0;
    int cycle2Path = 0;
    int cycle3Path = 0;
    int delay = 0;
    DriveBase.Path[] myPaths = null;
    int whitenext = 0;
    // int[] whitesclevels = { 15, 27, 27, 27, 27 };
    int[] whitesclevels = { 27, 32, 32, 32, 32 };

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        gpad.init();
        drive.init(hardwareMap);
        drive.initCamera(hardwareMap);
        drive.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myPaths = redPaths;
        
        while (!isStopRequested() && !opModeIsActive()) {
            gpad.update(gamepad1, gamepad2);
            drive.updateOTOS();

            double rx = -gpad.left_stick_y -gpad.right_stick_y;
            double ry = -gpad.right_stick_x;
            double rw = -gpad.left_stick_x;
            drive.driveXYW(rx, ry, rw, 0.5);
            
            if (gpad.guide) { bot.setMode(bot.mode.SETUP); }
            if (gpad.x && !gpad.previous.x) setAlliance(2, 1);
            if (gpad.b && !gpad.previous.b) setAlliance(1, 1);
            if (gpad.a && !gpad.previous.a) propPosition++;
            if (gpad.dpad_up && !gpad.previous.dpad_up) {
                cycle1Path++; cycle2Path = cycle1Path; cycle3Path = cycle1Path; 
            }
            if (gpad.dpad_left && !gpad.previous.dpad_left) cycle1Path++;
            if (gpad.dpad_down && !gpad.previous.dpad_down) cycle2Path++;
            if (gpad.dpad_right && !gpad.previous.dpad_right) cycle3Path++;
            if (gpad.y && !gpad.previous.y) setRobotStart();
            
            int delayinc = 0;
            if (gpad.right_bumper && !gpad.previous.right_bumper) delayinc = 1;
            if (gpad.shift.right_bumper && !gpad.previous.shift.right_bumper) delayinc = -1;
            delay = Range.clip(delay + delayinc, -30, 30);

            bot.update();
            int cpos = getRandomization();
            String clabel = (cpos < 0) ? "not seen" : propLabels[cpos % 3];
            propPosition = propPosition % 4;
            randomization = (cpos < 0) ? 3 : cpos % 3;
            String rlabel = clabel;
            String rformat = "%s (%d) from CAMERA";
            if (propPosition != 3) {
                randomization = propPosition;
                rlabel = propLabels[randomization];
                rformat = "%s (%d) FORCED";
            }
            drive.setFSpot(gpad.right_trigger >  0.5);

            purplePath = randomization + ((startPosition == 0) ? 0 : 3);
            yellowPath = randomization + startPosition * 3;
            cycle1Path %= 4;
            cycle2Path %= 4;
            cycle3Path %= 4;
            telemetry.addData("start", "%s %s", allianceLabels[alliance%3], startLabels[startPosition%3]);
            telemetry.addData("randomization", rformat, rlabel, randomization);
            telemetry.addData("last camera prop", "%s (%d)", clabel, cpos); 
            telemetry.addData(delay<0 ? "wait at START" : "wait after INTAKE", delay);
            telemetry.addData("purple", myPaths[purplePath].label);
            telemetry.addData("yellow", myPaths[yellowPath+6].label);
            telemetry.addData("cycle1", myPaths[cycle1Path+15].label);
            telemetry.addData("cycle2", myPaths[cycle2Path+15].label);
            telemetry.addData("cycle3", myPaths[cycle3Path+15].label);
            drive.addTelemetry(telemetry);
            drive.updateAprilTagPosition(telemetry);
            telemetry.update();
        }

        // driver pressed PLAY
        autonTimer.reset();

        logfile = new SimpleLogger();
        logfile.open();
        logfile.add("Phase").add(bot.csv(true)).add(drive.csv(true)).headerLine();
        drive.logfile = logfile;
        
        if (opModeIsActive()) {
            bot.grab(3);
            bot.launch(false);
            bot.spiv.setPosition(bot.SPIV_INIT);
            setRobotStart();
            waitFor(0.1);
        }
        
        if (delay < 0) waitFor(-delay);
        runStuff();
        
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
    
    void runStuff() {
        scorePurple(purplePath);
        scoreYellow(yellowPath);
        cycleWhite(0, cycle1Path);
        cycleWhite(1, cycle2Path);
        cycleWhite(2, cycle3Path);
        // park
        bot.setLevel(16);
        if (drive.field.x < 30) bot.setLevel(31);
        bot.resetMode(bot.mode.EXTEND);
    }
    
    public void setAlliance(int a, int i) {
        if (alliance != a) startPosition = 0;
        else startPosition += i;
        alliance = a % 3;
        startPosition = startPosition % 3;
        cycle1Path = (startPosition == 2) ? 2 : 0;
        cycle2Path = cycle1Path;
        cycle3Path = cycle1Path;
        setRobotStart();
    }
    
    public void setRobotStart() {
        alliance = alliance % 3;
        startPosition = startPosition % 3;
        bot.setLEDPattern(alliance);
        drive.setAlliance(alliance, (alliance == 1 ? 0 : 3));
        double startx;
        if (startPosition == 0) { // backdrop start
            startx = 16.5;
            drive.setAlliance(alliance, (alliance == 1 ? 0 : 3));
        } else { // audience start
            startx = -39;
            drive.setAlliance(alliance, (alliance == 2 ? 0 : 3));
        }
        if (alliance == 1) { 
            drive.setPosition(startx, 62.5, 90); 
            drive.setAlliance(alliance, startPosition == 0 ? 0 : 3);
        }
        if (alliance == 2) { 
            drive.setPosition(startx, -62.5, -90); 
            drive.setAlliance(alliance, startPosition == 0 ? 3 : 0);
        }
        myPaths = (alliance == 1) ? bluePaths : redPaths;
    }

    
    public int getRandomization() { 
        return drive.getPropPos();
    }

    public void scorePurple(int purplePath) {
        int[] scLevels;
        if (alliance == 1) scLevels = new int[]{20, 21, 22, 17, 18, 19};
        else scLevels = new int[]{17, 18, 19, 20, 21, 22};
        bot.setLevel(scLevels[purplePath]);
        bot.setMode(bot.mode.EXTEND);
        DriveBase.Pose p = myPaths[purplePath].get(0);
        double tx = p.x; double ty = p.y; double th = p.h;
        double vel = 0.5;
        moveTo(tx, ty, th, vel, purpleRamp);
        
        ElapsedTime purTimer = new ElapsedTime();
        
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            if (!drive.isHeading(th, 5)) purTimer.reset();
            if (purTimer.seconds() > 0.3) break;
            drive.driveXYW(0, 0, drive.calcRVW(th), 1);
        }
        drive.stop();     
        waitForIdle(2.5);
        bot.setMode(bot.mode.PURPLE);
        waitForIdle(5);
        bot.adjustTilt(1800);
        waitFor(0.1);
        bot.adjustTilt(0);
    }
    
    public void scoreYellow(int yellowPath) {
        double vel = 0.5;
        DriveBase.Path path = myPaths[yellowPath+6];
        int scLevel = 25;
        if (yellowPath >= 3) { // audience side, hold yellow pixel
            scLevel = 30;
            bot.setLevel(26);
            bot.setMode(bot.mode.EXTEND);
        }
        path.reset();
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            
            double dist = drive.followPath(path, yellowRamp);
            if (dist < 1) break;
            if (dist < 12) {
                bot.setLevel(scLevel);
                bot.setMode(bot.mode.EXTEND);
            }
            bot.update();
        }
        drive.stop();
        DriveBase.Pose pose = path.last();
        double tx = pose.x; double ty = pose.y; double th = pose.h;
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
                ry = (errY > 0) ? -0.15 : 0.15;
            }
            if (errX > 0) {
                navTimer.reset();
                rx = -0.2;
            }
            if (navTimer.seconds() > 0.3) break;
            drive.driveXYW(rx, ry, drive.calcRVW(180), 1);
            bot.update();
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
    
    public void cycleWhite(int seq, int cycleInPath) {
        if (autonTimer.seconds() > TIMEOUT_CYCLE_FULL) return;
        // pick cycle out path based on scoring path 
        int cycleOutPath = (cycleInPath < 2) ? 0 : 2;
        DriveBase.Path opath = myPaths[19 + cycleOutPath + seq/2];
        cycleOut(opath);
        distFlag = opath.last().h != 180 ? false : true;
        drive.setFSpot(true);
        pixelStrafeDance(opath.last());
        if (delay > 0 && seq == 0) waitFor( () -> autonTimer.seconds() > delay, delay);
        DriveBase.Path ipath = myPaths[15 + cycleInPath];
        int scLevel = 31; // floor score
        if (cycleInPath == 0 || cycleInPath == 2) {
            // backdrop score
            scLevel = whitesclevels[whitenext++]; 
        }
        cycleIn(ipath, scLevel);
        scoreWhite(ipath.last(), scLevel);
    }
    
    public void cycleOut(DriveBase.Path opath) {
        opath.reset();
        bot.intakeLevel = (opath.level > 0) ? 1 : 3;
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            
            double dist = drive.followPath(opath, cycleOutRamp);
            if (dist < 1) break;
            if (dist < 12) {
                bot.setMode(bot.mode.INTAKE);
            }
            bot.update();
        }
        drive.stop();
        opath.level++;
    }
    
    public void pixelStrafeDance(DriveBase.Pose pose) {
        double tx = pose.x; double ty = pose.y; double th = pose.h;
        double wdist = drive.getUSDist();
        double wallx = drive.field.x - wdist - 5.25;
        double wallxSum = wallx;
        double wallxCount = 1;
        boolean fwd = true;
        double fwdvel = 0.2;
        double bwdvel = 0.3;
        int danceStep = 0;
        double[] danceMoves = new double[]{ 0, -0.15, 0, 0.15, 0};
        ElapsedTime danceTimer = new ElapsedTime();
        double huskyOffset = 0;
        while (opModeIsActive()) {
            if (autonTimer.seconds() > TIMEOUT_RETRIEVE_PIXELS) break;
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);

            wdist = drive.getUSDist();
            wallxSum += drive.field.x - wdist - 5.25;
            wallxCount++;
            wallx = wallxSum / wallxCount;
            
            if (distFlag) {
                if (wdist < 4) fwd = false;
            }
            if (danceTimer.seconds() > 0.9) fwd = false;
            // if (wallx - drive.field.x < -15) fwd = false;
            
            double rvy = danceMoves[danceStep];
            if (fwd) {                
                HuskyLens.Block block = drive.getHusky();
                huskyOffset = 0;
                if (block != null && bot.mTimer.seconds() > 0.2) {
                    huskyOffset = (160-block.x);
                    rvy = huskyOffset * 0.31/80;
                }
                drive.driveXYW(fwdvel, rvy, drive.calcRVW(th), 1);
            } else if (drive.field.x > tx) {
                fwd = true;
                danceStep++;
                if (danceStep >= danceMoves.length) {
                    break;// bot.setMode(bot.mode.XFER);
                }
                danceTimer.reset();
                bot.intakeLevel = 1;
                bot.resetMode(bot.mode.INTAKE);
            } else {
                drive.moveToXYW(tx, ty, drive.calcRVW(th), bwdvel);
            }

            bot.update();
            if (!bot.isMode(bot.mode.INTAKE)) break;
            logfile.add("pixelStrafeDance").add(bot.csv())
                .add(drive.csv()).add(fwd).add(danceStep).add(wallx).add(wdist)
                .add(huskyOffset).tsLine();
        }
        drive.setFSpot(false);
        drive.stop();
        bot.setMode(bot.mode.XFER);
    }

    public void cycleIn(DriveBase.Path ipath, int scLevel) {
        ipath.reset();
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            bot.update();
            
            double dist = drive.followPath(ipath, cycleInRamp);
            if (dist < 0.5) break;
            if (dist < 24) { cycleExtend(scLevel); }
            drive.addTelemetry(telemetry);
            telemetry.update();
        }
        drive.stop();
        //bot.setMode(bot.mode.EXTEND);
        log("cyclein");
    }

    public boolean cycleExtend(int level) {
        if (bot.isMode(bot.mode.EXTEND) || bot.isMode(bot.mode.EXTENDED)) return true;
        if (bot.isMode(bot.mode.WAIT) || bot.isMode(bot.mode.SWING)) {bot.setLevel(level); bot.setMode(bot.mode.EXTEND); return false;}
        return false;
    }
    
    public void scoreWhite(DriveBase.Pose pose, int scLevel) {
        waitFor(() -> cycleExtend(scLevel), 3);
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
            if (waitTimer.seconds() > 3) break;
            double errX = tx - drive.field.x;
            double errY = ty - drive.field.y;
            if (!drive.isHeading(pose.h, 5)) navTimer.reset();
            //if (!bot.isMode(bot.mode.EXTENDED)) navTimer.reset();
            if (scLevel==31 && bot.isMode(bot.mode.EXTENDED)) break; // floor score
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
            
            log("white");
        }
        drive.stop();
        bot.setScoreSide(3);
        bot.setMode(bot.mode.SCORE);
        // waitFor( () -> !bot.isMode(bot.mode.SCORE) , 3);
        waitFor(0.5);
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
    

    DriveBase.Path[] redPaths = {
        // 0
        new DriveBase.Path("purple backdrop left")
                .addPose(12, -49.5, -60, 0.5),
        new DriveBase.Path("purple backdrop center")
                .addPose(15, -50, -90, 0.5),
        new DriveBase.Path("purple backdrop right")
                .addPose(17, -49.5, -120, 0.5),
        new DriveBase.Path("purple audience left")
                .addPose(-39, -50, -75, 0.35 ),
        new DriveBase.Path("purple audience center")
                .addPose(-38.5, -48.5, -118, 0.35),
        new DriveBase.Path("purple audience right")
                .addPose(-34.5, -51, -132.5, 0.35),

        // 6        
        new DriveBase.Path("yellow backdrop left")
                .addPose(42, -28, 180, 0.6),
        new DriveBase.Path("yellow backdrop center")
                .addPose(42, -33.5, 180, 0.6),
        new DriveBase.Path("yellow backdrop right")
                .addPose(42, -40, 180, 0.6),
        new DriveBase.Path("yellow door left")
                .addPose(-33, -35, -90, 0.3)
                .addPose(-33, -15, 180, 0.3)
                .addPose(-12, -11, 180, 0.6)
                .addPose( 13, -11, 180, 0.6)
                .addPose( 28, -15, 180-23, 0.6)
                .addPose( 42, -27.5, 180, 0.6),
        new DriveBase.Path("yellow door center")
                .addPose(-48, -35, -90, 0.3)
                .addPose(-48, -15, 180, 0.3)
                .addPose(-12, -11, 180, 0.6)
                .addPose( 13, -11, 180, 0.6)
                .addPose( 28, -15, 180-23, 0.6)
                .addPose( 42, -33.5, 180, 0.6),
        new DriveBase.Path("yellow door right")
                .addPose(-42, -47, -90, 0.3)
                .addPose(-42, -35, -90, 0.3)
                .addPose(-42, -15, 180, 0.3)
                .addPose(-12, -11, 180, 0.6)
                .addPose( 13, -11, 180, 0.6)
                .addPose( 28, -15, 180-23, 0.6)
                .addPose( 43, -39.5, 180, 0.6),
        new DriveBase.Path("yellow wall left")
                .addPose(-48, -48, 180, 0.3)
                .addPose(-36, -57, 180, 0.3)
                .addPose(14, -57, 180, 0.6)
                .addPose(32, -48, 180+20, 0.6)
                .addPose( 42, -28.5, 180, 0.6),
        new DriveBase.Path("yellow wall center")
                .addPose(-48, -48, 180, 0.3)
                .addPose(-36, -57, 180, 0.3)
                .addPose(14, -57, 180, 0.6)
                .addPose(32, -48, 180+20, 0.6)
                .addPose(43, -33.5, 180, 0.6),
        new DriveBase.Path("yellow wall right")
                .addPose(-48, -48, 180, 0.3)
                .addPose(-36, -57, 180, 0.3)
                .addPose(14, -57, 180, 0.6)
                .addPose(32, -48, 180+20, 0.6)
                .addPose(44, -39.5, 180, 0.6),

        // 15
        new DriveBase.Path("white door backdrop")
                .addPose(-56, -12, 180, 0.7)
                .addPose(11, -12, 180, 0.7)
                .addPose(37, -19, 180-20, 0.5),
        new DriveBase.Path("white door floor")
                .addPose(-48, -12, 180, 0.7)
                .addPose(11, -12, 180, 0.7)
                .addPose(20, -12, 180-10, 0.5),
        new DriveBase.Path("white wall backdrop")
                .addPose(-54, -48, 180-30, 0.5)
                .addPose(-36, -58, 180, 0.5)
                .addPose(11, -58, 180, 0.7)
                .addPose(38, -49, 180+15, 0.5),
        new DriveBase.Path("white wall floor")
                .addPose(-54, -48, 180-30, 0.5)
                .addPose(-36, -58, 180, 0.5)
                .addPose(11, -58, 180, 0.7)
                .addPose(20, -49, 180+0, 0.5),
        
        // 19
        new DriveBase.Path("white door stack 1")
                .addPose(36, -18, 180, 0.7)
                .addPose(24, -12, 180, 0.7)
                .addPose(-36, -12, 180, 0.7)
                .addPose(-56, -12, 180, 0.6),
        new DriveBase.Path("white door stack 2")
                .addPose(36, -15, 180, 0.7)
                .addPose(24, -12, 180, 0.7)
                .addPose(-45, -12, 180, 0.7)
                .addPose(-56, -22, 180+30, 0.6),
        new DriveBase.Path("white wall stack 1")
                .addPose(36, -48, 180, 0.6)
                .addPose(12, -58, 180, 0.6)
                .addPose(-36, -58, 180, 0.6)
                .addPose(-56, -42, 180-30, 0.6),
        new DriveBase.Path("white wall stack 2", 1)
                .addPose(36, -48, 180, 0.6)
                .addPose(12, -58, 180, 0.6)
                .addPose(-36, -58, 180, 0.6)
                .addPose(-56, -42, 180-30, 0.6),
    };

    DriveBase.Path[] bluePaths = {
        // 0 blue
        new DriveBase.Path("purple backdrop left")
                .addPose(15, 49.5, 120, 0.5),
        new DriveBase.Path("purple backdrop center")
                .addPose(15, 50, 90, 0.5),
        new DriveBase.Path("purple backdrop right")
                .addPose(11.5, 49.5, 50, 0.5),
        new DriveBase.Path("purple audience left")
                .addPose(-36.5, 49, 132.5, 0.35),
        new DriveBase.Path("purple audience center")
                .addPose(-41, 46.5, 118, 0.35),
        new DriveBase.Path("purple audience right")
                .addPose(-42.5, 48, 75, 0.35 ),

        // 6 blue        
        new DriveBase.Path("yellow backdrop left")
                .addPose(42, 42.5, 180, 0.6),
        new DriveBase.Path("yellow backdrop center")
                .addPose(42, 36.5, 180, 0.6),
        new DriveBase.Path("yellow backdrop right")
                .addPose(42, 31, 180, 0.6),
        new DriveBase.Path("yellow door left")
                .addPose(-42, 47, 90, 0.3)
                .addPose(-42, 35, 90, 0.3)
                .addPose(-42, 15, 180, 0.3)
                .addPose(-12, 15, 180, 0.6)
                .addPose( 13, 15, 180, 0.6)
                .addPose( 28, 15, 180+23, 0.6)
                .addPose( 42, 43.25, 180, 0.6),
        new DriveBase.Path("yellow door center")
                .addPose(-48, 35, 90, 0.3)
                .addPose(-48, 15, 180, 0.3)
                .addPose(-12, 15, 180, 0.6)
                .addPose( 13, 15, 180, 0.6)
                .addPose( 28, 15, 180+23, 0.6)
                .addPose( 42, 36.5, 180, 0.6),
        new DriveBase.Path("yellow door right")
                .addPose(-33, 35, 90, 0.3)
                .addPose(-33, 15, 180, 0.3)
                .addPose(-12, 15, 180, 0.6)
                .addPose( 13, 15, 180, 0.6)
                .addPose( 28, 15, 180+23, 0.6)
                .addPose( 43, 31.5, 180, 0.6),
        new DriveBase.Path("yellow wall left")
                .addPose(-48, 48, 180, 0.3)
                .addPose(-36, 57, 180, 0.3)
                .addPose(14, 57, 180, 0.6)
                .addPose(32, 48, 180-20, 0.6)
                .addPose( 42, 43.25, 180, 0.6),
        new DriveBase.Path("yellow wall center")
                .addPose(-48, 48, 180, 0.3)
                .addPose(-36, 57, 180, 0.3)
                .addPose(14, 57, 180, 0.6)
                .addPose(32, 48, 180-20, 0.6)
                .addPose(43, 36.5, 180, 0.6),
        new DriveBase.Path("yellow wall right")
                .addPose(-48, 48, 180, 0.3)
                .addPose(-36, 57, 180, 0.3)
                .addPose(14, 57, 180, 0.6)
                .addPose(32, 48, 180-20, 0.6)
                .addPose(44, 31.5, 180, 0.6),

        // 15 blue
        new DriveBase.Path("white door backdrop")
                .addPose(-48, 12, 180, 0.7)
                .addPose(11, 12, 180, 0.7)
                .addPose(36.5, 25.5, 180+10, 0.5),
        new DriveBase.Path("white door floor")
                .addPose(-48, 12, 180, 0.7)
                .addPose(11, 12, 180, 0.7)
                .addPose(20, 12, 180+10, 0.5),
        new DriveBase.Path("white wall backdrop")
                .addPose(-54, 48, 180+30, 0.5)
                .addPose(-36, 58, 180, 0.5)
                .addPose(11, 58, 180, 0.7)
                .addPose(38, 49, 180-15, 0.5),
        new DriveBase.Path("white wall floor")
                .addPose(-54, 48, 180+30, 0.5)
                .addPose(-36, 58, 180, 0.5)
                .addPose(11, 58, 180, 0.7)
                .addPose(20, 58, 180+0, 0.7),
        
        // 19 blue
        new DriveBase.Path("white door stack 1")
                .addPose(36, 18, 180, 0.7)
                .addPose(24, 13.5, 180, 0.7)
                .addPose(-36, 13.5, 180, 0.7)
                .addPose(-56, 13.5, 180, 0.6),
        new DriveBase.Path("white door stack 2")
                .addPose(36, 15, 180, 0.7)
                .addPose(24, 13.5, 180, 0.7)
                .addPose(-36, 13.5, 180, 0.7)
                .addPose(-56, 23, 180-20, 0.6),
        new DriveBase.Path("white wall stack 1")
                .addPose(36, 48, 180, 0.6)
                .addPose(12, 58, 180, 0.6)
                .addPose(-36, 58, 180, 0.6)
                .addPose(-56, 42, 180+30, 0.6),
        new DriveBase.Path("white wall stack 2", 1)
                .addPose(36, 48, 180, 0.6)
                .addPose(12, 58, 180, 0.6)
                .addPose(-36, 58, 180, 0.6)
                .addPose(-56, 42, 180+30, 0.6),
    };

}
