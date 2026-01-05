package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import android.graphics.Color;
import static org.firstinspires.ftc.teamcode.RobotValues.*;

public class RobotHardware extends DriveBase {
    public enum Phase { IDLE, SETUP, STOW, MANUAL,
        SEARCH, INTAKE, INTAKECLAW, OUTTAKE, GRAB, LIFT, 
        EXTEND, EXTEND_OBS, EXTEND_OBSNORTH, EXTEND_OBSWEST, EXTEND_BACK, EXTEND_FRONT, EXTEND45, 
        READY, RELEASE, 
        SPECSEARCH, SPECSEARCH_FRONT, SPECSCORE, RETRACT, SPECIDLE, 
        BASKET_NORTH, SWEEP, FORCE_RETRACT, AUTORETRACT, SUBDROP, CLAWMODE
    }
    public Phase phase;

    public final int OURS = 1;
    public final int NEUTRAL = 2;
    public final int THEIRS = 3;
    public final int NORTH = 0;
    public final int WEST = 1;
    public final int SOUTH = 2;
    public final int EAST = 3;
    
    public enum Task { IDLE, ABORTED, SETUP, SPEC_GRAB, SPEC_FRONT, SPEC_BACK, SPEC_BACK_TELE, SPEC_SCORE, SPEC_AUTOSCORE, 
                        GRABNGO, OBSNORTH, OBSWEST, BASKET, BASKETNORTH, BASKETEAST, BASKETEAST45, BASKETNORTH45, 
                        REJECT, SPEC_CYCLE, SUBDROP, LIMELIGHT_SEARCH, LIMELIGHT_GRAB }
    public Task task;
    public Task[][] grabngo = new Task[][]{ 
        // NONE, OURS, NEUTRAL, THEIRS
        //{Task.IDLE, Task.OBSNORTH , Task.BASKET, Task.REJECT} ,   //North
        //{Task.IDLE, Task.OBSWEST, Task.SUBDROP, Task.REJECT},   //West
        //{Task.IDLE, Task.IDLE, Task.IDLE, Task.REJECT},     //South
        //{Task.IDLE, Task.BASKET, Task.BASKET, Task.REJECT}    //East
        
        {Task.IDLE, Task.OBSNORTH , Task.BASKET, Task.REJECT} ,   //North
        {Task.IDLE, Task.OBSWEST, Task.SUBDROP, Task.REJECT},   //West
        {Task.IDLE, Task.IDLE , Task.IDLE, Task.REJECT},     //South
        {Task.IDLE, Task.BASKET, Task.BASKET, Task.REJECT}    //East
    };
    public int gngcount = 0;
    public int dtcount = 0;

    public final int ALLIANCERED = 1;
    public final int ALLIANCEBLUE = 3;
    public int alliance = ALLIANCERED;

    public ElapsedTime sampTimer; 
    public ElapsedTime wallTimer; 

    public ServoImplEx speclatch;
    public ServoImplEx specarm;
    public ServoImplEx specwrist;
    public ServoImplEx specgrab;
    public ServoImplEx grab;
    public ServoImplEx specswing;
    public ServoImplEx intakegrab;
    public ServoImplEx intakeswing;
    public ServoImplEx pushbar;
    public ServoImplEx trapdoor;
    public ServoImplEx clawwrist;
    public int clawwristpos = 0;

    public DigitalChannel wallf;
    public DigitalChannel wallb;

    public Extension slide;
    public DcMotorEx tilt;
    public DigitalChannel slidelimit;
    public DigitalChannel tiltlimit;
    public double lastSlideVel = 0;
    public double lastTiltVel = 0;

    public DigitalChannel sampcolor0;
    public DigitalChannel sampcolor1;
    public NormalizedColorSensor sampcolorrev; 
    public DistanceSensor sampdist; 
    public float hsvValues[] = new float[3];
    public int lastSampleColor;

    public ServoImplEx led5;
    public static int led5Mode = 0;
    public static double led5PWMPrev = 0;
    public static double led5PWMVal = 0;
    public static double led5PWMValUntil = 0;
    
    boolean pushbarOut = false;
    
    public AnalogInput bdist;
    public AnalogInput fdist;
    public AnalogInput specrange;
    
    public Limelight limelight;
    
    // boolean useSpecRange = false;

    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        speclatch = hardwareMap.get(ServoImplEx.class, "speclatch");
        speclatch.setDirection(Servo.Direction.FORWARD);
        specarm = hardwareMap.get(ServoImplEx.class, "specarm");
        specarm.setDirection(Servo.Direction.FORWARD);
        specwrist = hardwareMap.get(ServoImplEx.class, "specwrist");
        specwrist.setDirection(Servo.Direction.REVERSE);
        specgrab = hardwareMap.get(ServoImplEx.class, "specgrab");
        intakegrab = hardwareMap.get(ServoImplEx.class, "claw");
        intakeswing = hardwareMap.get(ServoImplEx.class, "intakeswing");
        trapdoor = hardwareMap.get(ServoImplEx.class, "wrist");
        intakeswing.setDirection(Servo.Direction.REVERSE);
        clawwrist = hardwareMap.get(ServoImplEx.class, "wrist");
        slide = new Extension(hardwareMap);
        // slide.setDirection(DcMotor.Direction.REVERSE);
        // slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt = hardwareMap.get(DcMotorEx.class, "tilt");
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt.setDirection(DcMotor.Direction.REVERSE);
        // tilt.setPositionPIDFCoefficients(25);
        tilt.setPositionPIDFCoefficients(20);
        slidelimit = hardwareMap.get(DigitalChannel.class, "slidelimit");
        tiltlimit = hardwareMap.get(DigitalChannel.class, "tiltlimit");
        pushbar = hardwareMap.get(ServoImplEx.class, "pushbar");
        // pushbar.setPwmRange(new PwmRange(550,2450));
        tilt.setPower(0);
        slide.setPower(0);
        sampcolor0 = hardwareMap.get(DigitalChannel.class, "sampcolor0");
        sampcolor1 = hardwareMap.get(DigitalChannel.class, "sampcolor1");
        sampcolorrev = hardwareMap.get(NormalizedColorSensor.class, "intakecolor");
        sampcolorrev.setGain((float)15);
        sampdist = hardwareMap.get(DistanceSensor.class, "intakecolor");
        sampTimer = new ElapsedTime(); 
        wallTimer = new ElapsedTime(); 
        
        led5 = hardwareMap.get(ServoImplEx.class, "led5");
        
        wallf = hardwareMap.get(DigitalChannel.class, "wallf");
        wallb = hardwareMap.get(DigitalChannel.class, "wallb");
        
        bdist = hardwareMap.get(AnalogInput.class, "bdist");
        fdist = hardwareMap.get(AnalogInput.class, "fdist");
        specrange = hardwareMap.get(AnalogInput.class, "specrange");
        
        dx.set("/specimen/intakeSide", SPECGRAB_BACK);
        dx.set("/robot/specscorex", 3.0);
        dx.set("/robot/specpickx", 43.0);
        dx.set("/robot/numBaskets", 0);
        dx.set("/robot/localizeSwitch", false);
        dx.set(HAS_SAMPLE, 0);

        dx.subscribe(SPECIMENPHASE, ph -> doSPECPHASE((Phase)ph));
        dx.subscribe(SAMPLEPHASE, ph -> doSAMPLEPHASE((Phase)ph));
        dx.subscribe("/dx/run", v -> update());
        dx.subscribe("/dx/run", run -> doLED((int)run));
        dx.subscribe("/dx/run", run -> detectSample()); 
        dx.subscribe("/dx/run", run -> updateWallTimer()); 
        dx.subscribe("/dx/run", run -> slide.update()); 
        dx.subscribe(ROBOTTASK, task -> doTASK((Task)task));
    }
    
    public void setAlliance(int a) {
        alliance = a;
        switch (alliance) {
            case ALLIANCERED: // red alliance
                ITDLocalize.setFieldRotation(0);
                setLED(0.28, 1);
                break;
            case ALLIANCEBLUE: // blue alliance
                ITDLocalize.setFieldRotation(180);
                setLED(0.61, 1);
                break;
        }
    }
    
    public void updateTracking() {
        super.updateTracking();
        ITDLocalize.updateAprilTagPosition(this);
    }
    
    public void initLimelight(HardwareMap hardwareMap) {
        limelight = new Limelight(hardwareMap);
    }
    
    public int sampleRevColor(double dist) {
        // 0 == none, 1 == red, 2 == yellow, 3 == blue
        Color.colorToHSV(sampcolorrev.getNormalizedColors().toColor(), hsvValues);
        double hue = hsvValues[0];
        if (hue >= 59 && hue <= 70) 
            return (dist < 2) ? 2 : 1;
        if (hue >= 200 && hue <= 250) return 3;
        if (hue >= 70 && hue <= 100) return 2;
        if (hue >= 320 || hue <= 60) return 1;
        return 0;
    }
    public int sampleType()
    {
        return sampleType(detectSample());
    }
    public int sampleType(int color) {
        if (alliance == ALLIANCERED) return color;
        if (color == 1) return THEIRS;
        if (color == 3) return OURS;
        return color;
    }
    
    public void updateWallTimer() {
        if (!onWall()) wallTimer.reset(); 
    }

    public int detectSample() {
        int color = 0;
        double dist = sampdist.getDistance(DistanceUnit.CM);
        if (dist > 3.2) {
            sampTimer.reset();
            lastSampleColor = 0;
            hsvValues[0] = -1;
            return 0;
        }
        color = sampleRevColor(dist);
        if (color != lastSampleColor) { sampTimer.reset(); }
        lastSampleColor = color;
        return (sampTimer.seconds() > 0.1) ? color : 0;
    }

    public boolean hasSample() {
        return sampTimer.seconds() > 0.1; 
    }
    
    public boolean hasSpecimen() {
        return specrange.getVoltage() < 0.17;
    }
    
    public boolean isPhase(String topic, Phase ph) {
        return (Phase)dx.get(topic) == ph;
    }
    
    public boolean isNorth() {
        double h = getHeading();
        return h >= -120 && h <= -45;
    }

    public int getFacing()
    {
        // 0 == north, 1 == west, 2 == south, 3 == east
        double h = getHeading();
        if (h >= -45 && h <= 45 ) return NORTH;     // north
        if (h >= 45 && h <= 135) return WEST;      // west
        if (h >= -135 && h <= -45) return EAST;    // east
        return SOUTH;                               // south
    }
    
    public void doSPECPHASE(Phase ph) {
        final String seqTopic = SPECIMENPHASE + "-seq";
        if (ph == Phase.IDLE){}; 
        if (ph == Phase.STOW) {
            speclatch.setPosition(SPECLATCH_DOWN);
            specgrab.setPosition(SPECGRAB_CLOSE);
            specarm.setPosition(SPECARM_STOW);
            specwrist.setPosition(SPECWRIST_STOW);
        }
        
        if (ph == Phase.RETRACT || ph == Phase.FORCE_RETRACT) {
            specarm.setPosition(SPECARM_HIGH + 0.1);
            speclatch.setPosition(SPECLATCH_UP);
            specgrab.setPosition(SPECGRAB_FRONT);
            dx.newSequence(seqTopic)
                .waitFor(0.2)
                .run(() -> specarm.setPosition(SPECARM_INTAKE))
                .waitFor(0.35)
                .run(() -> specwrist.setPosition(SPECWRIST_INTAKE))
                .run(() -> specgrab.setPosition((double) dx.get("/specimen/intakeSide")))
                .waitFor(0.4)
                .run(() -> speclatch.setPosition(SPECLATCH_DOWN))
                .set(SPECIMENPHASE, Phase.IDLE)
                .start();
        }
        if (ph == Phase.INTAKE) {
            specwrist.setPosition(SPECWRIST_INTAKE);
            speclatch.setPosition(SPECLATCH_DOWN);
            specgrab.setPosition((double) dx.get("/specimen/intakeSide"));
            dx.newSequence(seqTopic)
                .waitFor(0.5)
                .run(() -> specarm.setPosition(SPECARM_INTAKE))
                .start();
        }
        if (ph == Phase.GRAB) {
            specgrab.setPosition(SPECGRAB_CLOSE);
            speclatch.setPosition(SPECLATCH_DOWN);
            dx.newSequence(seqTopic)
                .waitFor(0.3)
                .run(() -> specwrist.setPosition(SPECWRIST_HIGH))
                .waitFor(0.1)
                .set(SPECIMENPHASE, Phase.EXTEND)
                .start();
        }
        if (ph == Phase.EXTEND) {
            specgrab.setPosition(SPECGRAB_CLOSE);
            speclatch.setPosition(SPECLATCH_DOWN);
            dx.newSequence(seqTopic)
                .run(() -> specarm.setPosition(SPECARM_HIGH))
                .waitFor(0.1)
                .set(SPECIMENPHASE, phase.READY)
                .start();
        }
    }

    public void setup() {
        dx.newSequence("/setup-seq")
            .set(SAMPLEPHASE, Phase.SETUP)
            .waitFor(SAMPLEPHASE, Phase.IDLE)
            .run(() -> tiltPosition(776))
            .start();
    }
    
    public boolean onWall() { return onWall(0b10000); }    
    public boolean onWall(int flags) {
        int state = (wallf.getState() ? 0b00100 : 0b00011) | (wallb.getState() ? 0b01000 : 0b10010);
        return (state & flags) == flags;
    }
    
    public void cancelTasks() {
        // if (dx.isEqual(ROBOTTASK, Task.BASKETNORTH)) {
        //     slidePosition(100);
        // }
        if (dx.isEqual(SAMPLEPHASE, Phase.EXTEND_BACK) || dx.isEqual(SAMPLEPHASE, Phase.EXTEND_FRONT)) {
            dx.set(SAMPLEPHASE, Phase.MANUAL);
            slidePosition(slide.getCurrentPosition());
            tiltPosition(tilt.getCurrentPosition());
        }
        if (!dx.isEqual(ROBOTTASK, Task.IDLE)) 
            dx.set(ROBOTTASK, Task.IDLE);
    }
    
    public void doTASK(Task task) {
        dtcount++;
        String topic = ROBOTTASK;
        String seqTopic = topic + "-seq";
        dx.cancelSequence(seqTopic);
        if (task == Task.SETUP) {
            dx.newSequence(seqTopic)
                .set(SAMPLEPHASE, Phase.SETUP)
                .waitFor(SAMPLEPHASE, Phase.IDLE)
                .run(() -> pushbar.setPosition(PUSHBAR_STOW))
                .run(() -> tiltPosition(1000))
                .set(ROBOTTASK, Task.IDLE)
                .start();
        }

        if (task == Task.SPEC_GRAB) {
            double p = 0.08 / 2;
            dx.newSequence(seqTopic)
                .set(SPECIMENPHASE, Phase.INTAKE)
                // .waitFor(() -> moveTargetXYH((double) dx.get("/robot/specpickx"), -66, 0, 0.2, 2))
                .waitFor(() -> driveToWallXYH(((double) dx.get("/robot/specpickx") - field.x) * p, -0.2, 0))
                .run(() -> stop())
                .set(topic, Task.SPEC_SCORE)
                .start();
        }
        if (task == Task.SPEC_FRONT) {
            dx.newSequence(seqTopic)
                .set("/specimen/intakeSide", SPECGRAB_FRONT)
                .set(SPECIMENPHASE, Phase.INTAKE)
                .waitFor(() -> driveToWallXYH(0, -0.2, 0))
                .waitFor(() -> {driveXYH(0.35, -0.04, 0); return false;}, 0.3)
                .run(() -> stop())
                .set(topic, Task.SPEC_SCORE)
                .start();
        }
        if (task == Task.SPEC_BACK) {
            doSpecBack(0.4, topic);
        }
        if (task == Task.SPEC_BACK_TELE) {
            doSpecBack(0.4, topic);
        }
        if (task == Task.SPEC_SCORE) {
            dx.newSequence(seqTopic)
                .set(SPECIMENPHASE, Phase.GRAB)
                .waitFor(SPECIMENPHASE, Phase.EXTEND)
                //0.6
                .waitFor(() -> moveTargetXYH((double)dx.get("/robot/specscorex")+3, -41, -5, 0.6, 2))
                .waitFor(SPECIMENPHASE, Phase.READY)
                .set(topic, Task.IDLE)
                .start();
        }
        if (task == Task.SPEC_AUTOSCORE) {
            double specscorex = (double)dx.get("/robot/specscorex");
            dx.newSequence(seqTopic)
                //0.8 0.55
                .waitFor(() -> moveTargetXYH(specscorex, -48, -5, 0.8, 3))
                .waitFor("/specimen/phase", Phase.READY)
                .waitFor(() -> moveTargetXYH(specscorex, -31, -5, 0.55, 2))
                .waitFor(0.1)
                .set(topic, Task.IDLE)
                .start();
        }
        if (task == Task.SUBDROP) {
            if (field.x < 0) {
                dx.set(ROBOTTASK, Task.IDLE);
                return;
            }
            dx.newSequence(seqTopic)
                .set(SAMPLEPHASE, Phase.SUBDROP)
                .set(ROBOTTASK, Task.IDLE)
                .start();
        }
        if (task == Task.GRABNGO) {
            gngcount++;
            int sc = sampleType();
            int facing = getFacing();
            // NONE, OURS, NEUTRAL, THEIRS
            // int facing = getFacing();   // NORTH, WEST, SOUTH, EAST
            
            dx.newSequence(seqTopic)
                .set(SAMPLEPHASE, Phase.INTAKECLAW)
                .waitFor(SAMPLEPHASE, Phase.LIFT)
                .waitFor(() -> !tilt.isBusy())
                .set(HAS_SAMPLE, (int) dx.get(HAS_SAMPLE)+1)
                .run(() -> dx.set(topic, grabngo[getFacing()][sampleType()]))
                .start();
        }
        if(task == Task.OBSNORTH) {
            dx.newSequence(seqTopic)
                .waitFor(() -> pathToOBSNORTH())
                .set(SAMPLEPHASE, Phase.EXTEND_OBSNORTH)
                .waitFor(() -> driveToWallXYH(0, -0.2, 0))
                .waitFor(() -> driveFieldXYW(0, -1, 0, 0.25), 0.5)
                .run(() -> stop())
                .waitFor(0.1)
                .set(ROBOTTASK, Task.IDLE)
                .start();
        }
        if(task == Task.OBSWEST) {
            // intakegrab.setPosition(INTAKE_SPEED);
            // dx.newSequence(seqTopic)
            //     .set(SAMPLEPHASE, Phase.LIFT)
            //     .waitFor(() -> slide.getCurrentPosition() < 265)
            //     .waitFor(() -> moveTargetXYH(53, -12, 90, 0.9, 3))
            //     .run(() -> slidePosition(SLIDE_OBSWEST))
            //     .run(() -> tiltPosition(TILT_SEARCH+40))
            //     .waitFor(() -> moveTargetXYH(55, -25, 180, RAMP80, 3, 3))
            //     .waitFor(() -> Math.abs(slide.getCurrentPosition() - SLIDE_OBSWEST) < 20)
            //     .set(ROBOTTASK, Task.IDLE)
            //     .start();
            
            dx.newSequence(seqTopic)
                .waitFor(() -> pathToOBSWEST())
                .set(SAMPLEPHASE, Phase.EXTEND_OBSWEST)
                .waitFor(0.1)
                .set(ROBOTTASK, Task.IDLE)
                .start();
        }
        if (task == Task.BASKET) {
            if (getFacing() == EAST){
                    dx.set(topic, Task.BASKETEAST45);
            }
            else
                dx.set(topic, Task.BASKETNORTH45);
        }
        if (task == Task.BASKETEAST45) {
            double fx = Math.min(field.x, -40);
            double fy = field.y;
            dx.newSequence(seqTopic)
                .waitFor(() -> slide.getCurrentPosition() < 265)
                .waitFor(() -> moveTargetXYH(fx, fy, -90, 0.9, 3))
                .set(SAMPLEPHASE, Phase.EXTEND45)
                .waitFor(() -> moveTargetXYH(-61, -52, -45, RAMP70, 3))
                .waitFor(() -> driveFieldXYW(-1, -1, calcRVW(-45), 0.25), 1)
                .run(() -> stop())
                .set(topic, Task.IDLE)
                .start();
        }
        if (task == Task.BASKETNORTH45) {
            double fy = Math.min(field.y, -40);
            dx.newSequence(seqTopic)
                .waitFor(() -> slide.getCurrentPosition() < 265)
                .waitFor(() -> moveTargetXYH(field.x, fy, 0, 0.9, 3))
                .set(SAMPLEPHASE, Phase.EXTEND45)
                .waitFor(() -> moveTargetXYH(-60, -54, -45, RAMP70, 3))
                .waitFor(() -> driveFieldXYW(-1, -1, calcRVW(-45), 0.25), 1)
                .run(() -> stop())
                .set(topic, Task.IDLE)
                .start();
        }
        if (task == Task.BASKETNORTH) {
            dx.newSequence(seqTopic)
                .waitFor(() -> slide.getCurrentPosition() < 265)
                .waitFor(() -> pathToBASKETNORTH()) 
                .set(SAMPLEPHASE, Phase.EXTEND_BACK)
                .run(() -> stop())
                .set(topic, Task.IDLE)
                .start();
        }
        if (task == Task.BASKETEAST) {
            Ramp r = new Ramp(1.0, 0.2, 0.6, 0.1);
            double fx = Range.clip(field.x, -66, -42);
            double fy = Range.clip(field.y,  -12, 12);
            dx.newSequence(seqTopic)
                .waitFor(() -> slide.getCurrentPosition() < 265)
                .waitFor(() -> moveTargetXYH(fx, fy, -90, 0.9, 3))
                .run(() -> tiltPosition(TILT_FRONT))
                .waitFor(() -> onWall() || moveTargetXYH(-69, Math.min(field.y, -36), -90, RAMP70, 3))
                .run(() -> stop())
                .waitFor(() -> tilt.getCurrentPosition() > TILT_FRONT * 0.7)
                .run(() -> intakeswing.setPosition(SWING_FRONT))
                .waitFor(() -> driveAlongWallXYH(r.scale(fdist.getVoltage()), -0.05, -90) || fdist.getVoltage() < 0.7)
                .run(() -> slidePosition(SLIDE_FRONT))
                .waitFor(() -> driveAlongWallXYH(r.scale(fdist.getVoltage()), -0.05, -90) || fdist.getVoltage() < 0.2)
                .run(() -> stop())
                .set(topic, Task.IDLE)
                .start();
        }
        if (task == Task.REJECT) {
            slide.setVelocity(0);
            dx.set(SAMPLEPHASE, Phase.IDLE);
            intakegrab.setPosition(INTAKE_REJECT);
            dx.newSequence(seqTopic)
                .waitFor(() -> !hasSample())
                .waitFor(0.5)
                .run(() -> intakegrab.setPosition(INTAKE_STOP))
                .set(topic, Task.IDLE)
                .start();
        }
        if (task == Task.SPEC_CYCLE) {
            double specscorex = 0.0;
            dx.newSequence(seqTopic)
                // SPEC_FRONT
                .waitFor(() -> moveTargetXYH(37, -62, 0, RAMP60, 3))
                .set("/specimen/intakeSide", SPECGRAB_FRONT)
                .set(SPECIMENPHASE, Phase.INTAKE)
                .waitFor(() -> driveToWallXYH(0, -0.2, 0))
                .run(() -> driveXYH(0.25, -0.04, 0))
                .waitFor(0.4)
                .run(() -> stop())
                .set(SPECIMENPHASE, Phase.GRAB)
                .waitFor(SPECIMENPHASE, Phase.EXTEND)
                // AUTOSCORE
                .waitFor(() -> moveTargetXYH(specscorex, -48, -5, 0.6, 3))
                .waitFor("/specimen/phase", Phase.READY)
                .waitFor(() -> moveTargetXYH(specscorex, -27.5, -5, 0.4, 1), 1)
                .waitFor(0.2)
                .waitFor(() -> moveTargetXYH(15, -50, 0, 0.6, 3))
                // return
                .set(SPECIMENPHASE, Phase.RETRACT)
                .waitFor(() -> moveTargetXYH(37, -62, 0, RAMP60, 2))
                .set(ROBOTTASK, Task.IDLE)
                .start();
        }

        if (task == Task.LIMELIGHT_SEARCH) {
            if (limelight == null) {
                dx.set(ROBOTTASK, Task.IDLE);
                return;
            }
            dx.set(SAMPLEPHASE, Phase.SEARCH);
            dx.newSequence(seqTopic)
                .waitFor(() -> limelight.processTargetPos(), 1)
                .set(ROBOTTASK, Task.LIMELIGHT_GRAB)
                .start();
        }
        
        if (task == Task.LIMELIGHT_GRAB) {
            dx.set(SAMPLEPHASE, Phase.SEARCH);
            Limelight.LLDetection targetPos = limelight.targetPos;
            if (targetPos == null) {
                dx.set(ROBOTTASK, Task.IDLE);
                return;
            }
            goPosition(slide, targetPos.slide, 0.6);
            slideTilt(targetPos.slide);
            clawwrist.setPosition(targetPos.angle);

            double th = getHeading();
            double tx = field.x + targetPos.xDist * Math.cos(Math.toRadians(th));
            double ty = field.y + targetPos.xDist * Math.sin(Math.toRadians(th));
            
            dx.newSequence(seqTopic)
                .waitFor(() -> moveTargetXYH(tx, ty, th, 0.2, 1))
                .waitFor(() -> Math.abs(slide.getCurrentPosition() - targetPos.slide) < 10)
                .waitFor(0.5)
                .set(SAMPLEPHASE, Phase.INTAKECLAW)
                .waitFor(SAMPLEPHASE, Phase.LIFT)
                .set(ROBOTTASK, Task.IDLE)
                .start();
        }
    }
    
    public void doSAMPLEPHASE(Phase ph) {
        String seqTopic = SAMPLEPHASE + "-seq";
        dx.cancelSequence(seqTopic);
        // if (ph == Phase.SEARCH || (ph == Phase.MANUAL && tilt.getCurrentPosition() < 500))
        //     slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // else 
        //     slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                
        if (ph == Phase.SEARCH) {
            if (tilt.getCurrentPosition() > TILT_SEARCH_MAX)
                slidePosition(SLIDE_LIFT);
            intakeswing.setPosition(SWING_DOWN);
            intakegrab.setPosition(CLAW_OPEN);
            // trapdoor.setPosition(TRAPDOOR_SEARCH);
            tiltPosition(TILT_SEARCH);
            dx.newSequence(seqTopic)
                .waitFor(() -> inRect(-10, 12.5, -25, -33) && isHeading(0, 5))
                .run(() -> slidePosition((int)(SLIDE_HORIZ_LIMIT*0.25)))
                .start();
        }
        if(ph == Phase.INTAKE){
            if (tilt.getCurrentPosition() > TILT_SEARCH_MAX)
                slidePosition(SLIDE_LIFT);
            intakegrab.setPosition(INTAKE_SPEED);
            intakeswing.setPosition(SWING_DOWN);
            trapdoor.setPosition(TRAPDOOR_INTAKE);
            dx.setWhen(() -> hasSample(), SAMPLEPHASE, Phase.LIFT); 
        }
        if(ph == Phase.INTAKECLAW) {
            if (hasSample()) {
                dx.set(SAMPLEPHASE, Phase.LIFT);
                return;
            }
            adjustSlide(0);
            intakegrab.setPosition(CLAW_OPEN);
            dx.newSequence(seqTopic)
                .waitFor(0.3)
                .run(() -> intakegrab.setPosition(CLAW_CLOSE))
                .waitFor(0.3)
                .set(SAMPLEPHASE, Phase.LIFT)
                .start();
        }
        if(ph == Phase.CLAWMODE){
            intakegrab.setPosition(0.392);
            
            //tiltPosition(TILT_SEARCH);
            intakegrab.setPosition(494);
            dx.setWhen(() -> hasSample(), SAMPLEPHASE, Phase.LIFT); 
        }
        if(ph == Phase.LIFT){
            intakeswing.setPosition(SWING_LIFT);
            // trapdoor.setPosition(TRAPDOOR_CLOSE);
            tiltPosition(TILT_LIFT);
            dx.newSequence(seqTopic)
                .waitFor(() -> tilt.getCurrentPosition() > TILT_LIFT-50)
                .run(() -> slidePosition(SLIDE_LIFT))
                .start();
        }
        if (ph == Phase.EXTEND) {
            
            if (isNorth()) {
                dx.set(SAMPLEPHASE, Phase.EXTEND_FRONT);
            }
            else
                dx.set(SAMPLEPHASE, Phase.EXTEND_BACK);
        }
        if (ph == Phase.EXTEND_OBSNORTH || ph == Phase.EXTEND_OBS) {
            slidePosition(SLIDE_OBSNORTH);
            tiltPosition(TILT_OBSNORTH);
            intakeswing.setPosition(SWING_OBSNORTH);
            clawwrist.setPosition(CLAW_WRIST_OBSNORTH);
            // trapdoor.setPosition(TRAPDOOR_HOLD);
        }
        if (ph== Phase.EXTEND_OBSWEST) {
            slidePosition(SLIDE_OBSWEST);
            tiltPosition(TILT_OBSWEST);
            intakeswing.setPosition(SWING_OBSWEST);
            // trapdoor.setPosition(TRAPDOOR_HOLD);
        }
        if (ph == Phase.EXTEND_BACK) {
            trapdoor.setPosition(TRAPDOOR_CLOSE);
            dx.newSequence(seqTopic)
              .run(() -> tiltPosition(TILT_BACK))
              .waitFor(() -> tilt.getCurrentPosition() > TILT_BACK * 0.5)
              .run(() -> slidePosition(SLIDE_BACK))
              .run(() -> intakeswing.setPosition(SWING_BACK))
              .waitFor(() -> !slide.isBusy())
             
              .start();
        }
        if (ph == Phase.EXTEND_FRONT) {
            trapdoor.setPosition(TRAPDOOR_CLOSE);
            dx.newSequence(seqTopic)
              .run(() -> tiltPosition(TILT_FRONT))
              .waitFor(() -> tilt.getCurrentPosition() > TILT_FRONT * 0.7)
              .run(() -> slidePosition(SLIDE_FRONT))
              .run(() -> intakeswing.setPosition(SWING_FRONT))
              .waitFor(() -> !slide.isBusy())
              .start();
        }
        if (ph == Phase.EXTEND45) {
            //trapdoor.setPosition(TRAPDOOR_CLOSE);
            dx.newSequence(seqTopic)
              .run(() -> goPosition(tilt, TILT45, 1))
              .waitFor(() -> tilt.getCurrentPosition() > TILT45 * 0.8 && (field.x < -36 || field.y < -36))
              .run(() -> slidePosition(SLIDE45))
              .run(() -> intakeswing.setPosition(SWING180))
              .run(() -> clawwrist.setPosition(CLAW_SCORE))
              .start();
        }
        if(ph == Phase.RELEASE) {
            // if (inRect(35, 74, -62, -74)) {
            //     trapdoor.setPosition(TRAPDOOR_OBSZONE);
            //     intakegrab.setPosition(isHeading(90, 5) ? INTAKE_REJECT : INTAKE_RELEASE_OBS);
            // } else if (tilt.getCurrentPosition() > 1000 && inRect(-74, -30, -30, -74)) {
            //     int baskets = (int)dx.get("/robot/numBaskets");
            //     intakegrab.setPosition(baskets > 9 ? INTAKE_RELEASE_SLOW : INTAKE_RELEASE);
            //     dx.set("/robot/numBaskets", baskets+1);
            // } else
            //     intakegrab.setPosition(INTAKE_RELEASE);
            intakegrab.setPosition(CLAW_OPEN);
            dx.newSequence(seqTopic)
                // .waitFor(() -> !hasSample())
                .waitFor(0.3)
                // .run(() -> intakegrab.setPosition(INTAKE_STOP))
                .set(SAMPLEPHASE, Phase.RETRACT)
                .start();
        }
        if (ph == Phase.RETRACT) {
            trapdoor.setPosition(TRAPDOOR_CLOSE);
            intakegrab.setPosition(INTAKE_STOP);
            boolean slideUp = slide.getCurrentPosition() > 400;
            // boolean tiltUp = tilt.getCurrentPosition() > 1300;
            if (slideUp) {
                intakeswing.setPosition(SWING_RETRACT);
                slidePower(0.63); //0.67
            }
            dx.newSequence(seqTopic)
                .waitFor(slideUp ? 0.3 : 0) //0.3
                .run(() -> slidePower(-0.8))
                .run(() -> goPosition(tilt, TILT_LIFT, field.x < -12 ? 0.5 : 0.75))
                .waitFor(() -> !slidelimit.getState())
                .run(() -> slidePower(0))
                .run(() -> slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER))
                .set(SAMPLEPHASE, Phase.LIFT)
                .start();
        }
        if (ph == Phase.AUTORETRACT) {
            intakeswing.setPosition(SWING_RETRACT);
            slidePower(0.6);//0l.6
            tiltPower(0);
            dx.newSequence(seqTopic)
                .waitFor(0.3)
                .run(() -> slidePower(-0.8))
                .waitFor(() -> !slidelimit.getState())
                .run(() -> slidePower(0))
                .run(() -> slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER))
                .set(SAMPLEPHASE, Phase.LIFT)
                .start();
        }
        if (ph == Phase.SUBDROP) {
            dx.newSequence(seqTopic)
                .run(() -> tiltPosition(TILT_SEARCH))
                .run(() -> slidePosition(SLIDE_MAX))
                .start();
        }
        if(ph == Phase.OUTTAKE) {
            intakegrab.setPosition(INTAKE_REJECT);
            dx.setWhen(0.5, SAMPLEPHASE, Phase.IDLE);
        }
        // if (ph == Phase.PUSH_CLEAR) {
        //     dx.newSequence(seqTopic)
        //         .run(() -> pushbar.setPosition(PUSHBAR_PUSH))
        //         .waitFor(0.5)
        //         .run(() -> pushbar.setPosition(PUSHBAR_READY))
        //         .start();
        // }
        if (ph == Phase.IDLE) {
            // if (!hasSample()) intakegrab.setPosition(INTAKE_STOP);
        }
        if (ph == Phase.SWEEP) {
            intakeswing.setPosition(SWING_DOWN);
            tiltPosition(TILT_SEARCH);
            slidePosition(SLIDE_SWEEP);
            dx.newSequence(seqTopic)
                .waitFor(0.2)
                .run(() -> tiltPosition(TILT_SWEEP))
                .run(() -> intakegrab.setPosition(INTAKE_SPEED))
                .waitFor(0.1)
                .run(() -> goPosition(slide, SLIDE_HORIZ_LIMIT, 0.4))
                .waitFor(() -> hasSample())
                .run(() -> tiltPosition(TILT_SEARCH))
                .run(() -> slidePosition(SLIDE_SWEEP))
                .set(SAMPLEPHASE, Phase.READY)
                .start();
        }
        if (ph == Phase.SETUP) {
            trapdoor.setPosition(TRAPDOOR_CLOSE);
            dx.newSequence(seqTopic)
                .run(() -> tiltPower(0.4))
                .waitFor(0.5)
                .run(() -> intakeswing.setPosition(SWING_SETUP))
                .run(() -> tiltPower(0))
                .run(() -> slidePower(-0.4))
                .waitFor(() -> !slidelimit.getState())
                .run(() -> slidePower(0))
                .run(() -> tiltPower(-0.4))
                .waitFor(() -> !tiltlimit.getState())
                .run(() -> tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER))
                .run(() -> tiltPosition(TILT_LIFT))
                .waitFor(() -> !tilt.isBusy())
                .run(() -> tiltPower(0))
                .run(() -> slidePower(-0.4))
                .waitFor(() -> !slidelimit.getState())
                .run(() -> slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER))
                .run(() -> slidePower(0))
                .run(() -> intakeswing.setPosition(SWING_DOWN))
                .set(SAMPLEPHASE, Phase.IDLE)
                .start();
        }
    }
    
    public void update() {
        // when ready to score specimens, rotate wrist to match heading
        if (dx.isEqual(SPECIMENPHASE, Phase.READY)) {
            double h = Range.clip(getHeading(), -25, 25);
            specwrist.setPosition(SPECWRIST_HIGH + h * SPECWRIST_DEGREE);
        }
        if (dx.isEqual(SAMPLEPHASE, Phase.INTAKE)) {
            double offset = Math.sin(dx.clock.seconds()*8) * 0.075;
            intakeswing.setPosition(SWING_DOWN);
            offset = ((dx.clock.seconds() % 0.75) < 0.45) ? 0 : -0.25;
            trapdoor.setPosition(TRAPDOOR_INTAKE + offset);
        }
    }

    public void adjustSlide(double vel) {
        int slidepos = slide.getCurrentPosition();
        slideTilt(slidepos);
        if (vel == 0) {
            if (lastSlideVel != 0) {
                slide.setTargetPosition(slidepos, 0.5);
            }
            lastSlideVel = vel;
            return;
        }
        lastSlideVel = vel;
        // maxvel controls maximum horizontal reach
        int maxvel = (SLIDE_VERT_LIMIT - slidepos) * 3;
        if (vel > maxvel) vel = maxvel;
        maxvel = (SLIDE_HORIZ_LIMIT - slidepos) * 3;
        if (vel > maxvel && tilt.getCurrentPosition() < 1500) 
            vel = maxvel;
        if (vel < 0 && !slidelimit.getState()) 
            vel = 0;
        if (dx.isEqual(SAMPLEPHASE, Phase.INTAKE) || dx.isEqual(SAMPLEPHASE, Phase.SEARCH))
            vel = Range.clip(vel, -2500, 2500); 
        else
            dx.set(SAMPLEPHASE, Phase.MANUAL);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setVelocity(vel);
    }
    
    public void slideTilt(int slidepos) {
        Phase ph = (Phase)dx.get(SAMPLEPHASE);
        double tiltpos = 0;
        if (ph == Phase.SEARCH) tiltpos = Range.scale(slidepos, SLIDE_LIFT, SLIDE_HORIZ_LIMIT, TILT_SEARCH, 400);
        if (ph == Phase.INTAKE) tiltpos = Range.scale(slidepos, SLIDE_LIFT, SLIDE_HORIZ_LIMIT, 170, 370); //170,390
        if (ph == Phase.INTAKECLAW) tiltpos = Range.scale(slidepos, SLIDE_LIFT, SLIDE_HORIZ_LIMIT, 140, 330); //170,390
        if (tiltpos < 1) return;
        tiltPosition((int)tiltpos);
    }

    public void adjustTilt(double vel) {
        int tiltpos = tilt.getCurrentPosition();
        if (vel == 0) {
            if (lastTiltVel != 0) {
                tilt.setTargetPosition(tiltpos);
                tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                tilt.setPower(0.5);
            }
            lastTiltVel = vel;
            return;
        }
        lastTiltVel = vel;
        tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilt.setVelocity(vel);
        
        // if (!isPhase(SAMPLEPHASE, Phase.LIFT))
        dx.set(SAMPLEPHASE, Phase.MANUAL);
    }

    public void slidePower(double p) {
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setPower(p);
    }
    public void tiltPower(double p) {
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt.setPower(p);
    }
    
    public void slidePosition(int pos) { goPosition(slide, pos, 0.9); }
    public void tiltPosition(int pos) { goPosition(tilt, pos, 0.5); }
    
    public void goPosition(Extension e, int pos, double pow) {
        e.setTargetPosition(pos, pow);
    }
    public void goPosition(DcMotor m, int pos, double pow) {
        m.setTargetPosition(pos);
        if (m.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setPower(pow);
    }
    
    public void setClawWrist(int pos) {
        clawwristpos = pos % CLAW_WRIST.length;
        clawwrist.setPosition(CLAW_WRIST[clawwristpos]);
    }
    
    public void nextClawWrist() {
        setClawWrist(clawwristpos+1);
    }

    
    public boolean driveToWallXYH(double rx, double ry, double fh) {
        if (onWall()) { stop(); return true; }
        driveXYH(rx, ry, fh);
        return false;
    }
    
    public boolean driveAlongWallXYH(double rx, double ry, double fh) {
        if (!onWall()) { ry = -0.2; }
        driveXYH(rx, ry, fh);
        return false;
    }
    
    public boolean pathToOBSNORTH() {
        if (inRect(38, 74, -62, -74) && isHeading(0, 5)) { // we're in the OZ
            dx.set(SAMPLEPHASE, Phase.EXTEND_OBSNORTH);
            stop();
            return true;
        }
        if (field.x > 41 || field.y < -42) { // clear path to OZ
            dx.set(SAMPLEPHASE, Phase.EXTEND_OBSNORTH);
            moveTargetXYH(40, -66.5, 0, RAMP80, 2);
            return false;
        }
        if (field.x > -18 && field.y < 0) { // near our chambers, so back away from submersible
            moveTargetXYH(Range.clip(field.x, -8, 12), -64, 0, 0.6, 3);
            return false;
        }
        // driver figures it out
        return false;
    }

    public boolean pathToOBSWEST() {
        DriveBase.Ramp presetrampwest = new DriveBase.Ramp(16, 8, 0.7, 0.2);
        if (inRect(56, 74, -74, -41) && moveTargetXYH(69, -57, 90, presetrampwest, 3)) { // robot is already in oz
            dx.set(SAMPLEPHASE, Phase.EXTEND_OBSWEST);
            stop();
            return true;
        }
        if (field.x > 48 || field.y < -24) { // robot has clear path to oz
            dx.set(SAMPLEPHASE, Phase.EXTEND_OBSWEST);
            moveTargetXYH(70, -58, 90, presetrampwest, 3);
            return false;
        }
        if (field.x > 0) { // robot is near opponent rungs, so move away from sub
            dx.set(SAMPLEPHASE, Phase.EXTEND_OBSWEST);
            moveTargetXYH(61.5, -18, 90, 0.9, 3);
            return false;
        }
        // no automatic path so let driver do it
        return false;
    }
    
    public boolean pathToBASKETNORTH() {
        double bv = bdist.getVoltage();        
        if (inRect(-74, -56, -74, -62) && bv < 0.08 && isHeading(0, 5)) { // in scoring position
            stop();
            return true;
        }
        if (onWall() || field.y < -66) { // against wall
            dx.set(SAMPLEPHASE, Phase.EXTEND_BACK);
            Ramp r = new Ramp(1.0, 0.2, -0.6, -0.1);
            driveAlongWallXYH(r.scale(bv), -0.05, 1);
            return false;
        }
        if (field.x < -42 || field.y < -42) { // clear of submersible
            dx.set(SAMPLEPHASE, Phase.EXTEND_BACK);
            moveTargetXYH(-36, -68, 1, 0.8, 3);
            return false;
        }
        if (field.y < 0 && field.x < 11) { // in submersible
            moveTargetXYH(-9, -50, 1, 0.8, 3);
            return false;
        }
        // driver-control needed
        return false;
    }
    
    public void doSpecBack(double time, String topic) {
        String seqTopic = topic + "-seq";
        dx.newSequence(seqTopic)
            .set("/specimen/intakeSide", SPECGRAB_BACK)
            .set(SPECIMENPHASE, Phase.INTAKE)
            .waitFor(() -> driveToWallXYH(0, -0.2, 0))
            .waitFor(() -> {driveXYH(-0.35, -0.04, 0); return false;}, time)
            .run(() -> stop())
            .set(topic, Task.SPEC_SCORE)
            .start();
    }
    
    public void setLED(double pos, double sec) {
        led5PWMVal = pos;
        led5PWMValUntil = dx.clock.seconds() + sec;
    }
    
    public void doLED(int run) {
        double led5PWM = 0;
        if (dx.clock.seconds() < led5PWMValUntil) led5PWM = led5PWMVal;
        switch (led5Mode % 3) {
            case 1:
                int r = run % 200;
                if (r > 99) r = 200 - r;
                led5PWM = Range.scale(r, 0, 99, 0.280, 0.720);
                break;
            case 2:
                int sc = detectSample();
                if (sc == 1) led5PWM = 0.280; // red
                if (sc == 2) led5PWM = 0.388; // yellow
                if (sc == 3) led5PWM = 0.611; // blue
                break;
        }
        if (slide.slide1.getCurrent(CurrentUnit.AMPS) > 5) led5PWM = 0.333;

        if (led5PWM != led5PWMPrev) led5.setPosition(led5PWM);
        led5PWMPrev = led5PWM;
    }
    
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("gngcount", gngcount);
        telemetry.addData("dtcount", dtcount);
        telemetry.addData("alliance", alliance == ALLIANCERED ? "red" : "blue");
        telemetry.addData("numBaskets", (int) dx.get("/robot/numBaskets"));
        telemetry.addData("bdist", bdist.getVoltage());
        telemetry.addData("fdist", fdist.getVoltage());
        telemetry.addData("specrange", specrange.getVoltage());
        telemetry.addData("slide", slide.toString());
        telemetry.addData("tilt", TelemetryEx.format(tilt));
        telemetry.addData("tiltlimit", tiltlimit.getState());
        telemetry.addData("slidelimit", slidelimit.getState());
        telemetry.addData("sampdist", sampdist.getDistance(DistanceUnit.CM));
        telemetry.addData("samplecolor", detectSample());
        telemetry.addData("sampleTimer", sampTimer.seconds());
        telemetry.addData("hue", hsvValues[0]);
        telemetry.addData("isNorth", isNorth());
        telemetry.addData("facing" , getFacing());
        super.addTelemetry(telemetry);
    }
}