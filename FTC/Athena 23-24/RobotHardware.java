package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotValues.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.openftc.i2cdrivers.AdafruitLEDBackpack;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class RobotHardware {
    public Servo gantry;
    private Servo larm;
    public Servo lscore;
    private DcMotorEx ltilt;
    public DcMotorEx intake;
    private DcMotorEx rtilt;
    private DcMotorEx elev;
    private Servo rarm;
    public Servo rscore;
    public DigitalChannel lball;
    public DigitalChannel rball;
    private Servo rhook;
    private Servo lhook;
    public Servo spiv;
    private DigitalChannel elim;
    private DigitalChannel rtlim;
    private DigitalChannel ltlim;
    private NormalizedColorSensor lcolor;
    private NormalizedColorSensor rcolor;
    private DistanceSensor ldistance;
    private DistanceSensor rdistance;
    private Blinker cLed;
    private Blinker eLed;
    public AdafruitLEDBackpack matrix;
    public Servo launcher; 

    public final double INTAKE_IN = 1.0;
    public final double INTAKE_HOLD = 0.3;
    public final double INTAKE_XFER = .6;
    public final double INTAKE_OUT = -0.4;
    public final double LARM_STOW = 0.740;
    public final double LARM_LEV0 = 0.150;
    public final double LARM_LEV3 = 0.210;
    public final double LARM_LEV4 = 0.53;
    public final double LARM_HOLD = 0.235;
    public final double LARM_XFER = LARM_STOW;
    //0.55
    public final double LARM_SETUP = LARM_STOW;
    public final double RARM_STOW = 0.780;
    public final double RARM_LEV0 = 0.390;
    public final double RARM_LEV3 = 0.435;
    public final double RARM_LEV4 = 0.55;
    public final double RARM_HOLD = 0.5;
    public final double RARM_XFER = RARM_STOW;
    //0.57
    public final double RARM_SETUP = RARM_STOW;
    public final double GANTRY_XFER = 0.84;
    //.74
    public final double GANTRY_SCORE = 0.28;
    // public final double GANTRY_PURPLE = 0.40;
    public final double GANTRY_SWING = 0.55;
    public final double GANTRY_STOW = GANTRY_SWING;
    public final double GANTRY_SETUP = GANTRY_STOW;
    public final double LSCORE_OPEN = 0.695;
    public final double LSCORE_CLOSE = 0.433;
    public final double RSCORE_OPEN = 0.575;
    public final double RSCORE_CLOSE = 0.32;
    public final double SPIV_XFER = 0.25;
    //.275
    public final double SPIV_STOW = 0.175;
    public final double SPIV_INIT = 0.18;
    public final double SPIV_SWING = 0.400;
    //.6
    public final double SPIV_SCORE = 0.600;
    public final double SPIV_MP = 0.1; 
    public final double SPIV_RETRACT = 0.680;
    public final double SPIV_PURPLE = 0.40;
    public final double SPIV_CENTER_SCORE = 0.5;
    public final double SPIV_SETUP = 0.25;
    public final double TILT_VELOCITY = 2600;
    public final double TILT_DOWN_VELOCITY = 700;
    public final double TILT_RETRACT_VELOCITY = -1500;
    public final double TILT_CLIMB_VELOCITY = -2600;
    public final double ELEV_VELOCITY = 1900;
    public final double ELEV_RETRACT_VELOCITY = -1500;
    public final double LHOOK_UP = 0.665;
    public final double RHOOK_UP = 0.37;
    public final double LHOOK_DOWN = 0.22;
    public final double RHOOK_DOWN = 0.8;
    public final double TURN_GAIN = 0.03; //0.05
    public final double RANGE_GAIN = 0.05; //0.02
    public final double YAW_GAIN = 0.03; //0.05

    public final int TELEOP_LEVEL_MIN = 1;
    public final int TELEOP_LEVEL_MAX = 10;
    
    public final double LAUNCHER_OPEN = 0.525;
    public final double LAUNCHER_CLOSE = 0.41;

    public final double[][] elevPos = {
        //tilt, elev, spiv
        {50, 26, SPIV_STOW}, // 0: stow
        {245, 300, SPIV_SCORE}, //level 1
        {349, 400, SPIV_SCORE}, //level 2
        {481, 520, SPIV_SCORE-0.04}, //level 3
        {572, 510, SPIV_SCORE-0.06}, //level 4
        {683, 530, SPIV_SCORE-0.08}, //level 5
        {780, 590, SPIV_SCORE-0.10}, //level 6
        {858, 650, SPIV_SCORE-0.10}, //level 7
        {858, 650, SPIV_SCORE-0.10}, //level 8
        {858, 650, SPIV_SCORE-0.16}, //level 9
        {1190, 670, SPIV_SCORE-0.16}, //level 10
        {50, 26, SPIV_SETUP},           // 11: setup
        {50, 26, SPIV_XFER},            // 12: transfer
        {1915, 26, SPIV_SETUP},         // 13: hang mode
        //1885
        {189, 380, SPIV_SCORE},         // 14: auton yellow score
        {350, 715, SPIV_SCORE + 0.1},    // 15: auton white score 
        {33, 200, SPIV_STOW},          // 16: auton park
        {33, 370, SPIV_PURPLE},         // 17: redback/blueaud left purple
        {33, 530, SPIV_PURPLE},         // 18: redback/blueaud center purple
        {33, 290, SPIV_PURPLE},         // 19: auton 6 purple
        {33, 270, SPIV_PURPLE},         // 20: auton 1 purple
        {33, 530, SPIV_PURPLE},         // 21: auton 2 purple
        {33, 440, SPIV_PURPLE},         // 22: blueback / redaud right purple
        {49, 26, SPIV_SETUP},           // 23
        {33, 557, SPIV_PURPLE},         // 24: audience 6 purple
        {292, 410, SPIV_SCORE},          // 25: audience yellow score
        {33, 30, SPIV_SCORE + 0.1},     // 26: audience yellow hold
        {400, 730, SPIV_SCORE},          // 27: auton white high score 
        {350, 730, SPIV_SCORE + 0.1},   // 28: auton white mid score
        {260, 380, SPIV_SCORE},           // 29: offset yellow score
        {400, 380, SPIV_SCORE},         // 30: audience yellow score
        {100, 730, SPIV_SCORE},         // 31: auton white floor score
        {450, 760, SPIV_SCORE},          // 32: auton white highest score 
    };

    public enum Mode { IDLE, STOW, SETUP, 
        INTAKE, OUTTAKE, INTAKEIDLE, HOLD,
        XFER, SWING, EXTEND, EXTENDED,
        SCORE, RETRACT, REACH, HANG, LAUNCH, CLIMB, PURPLE,
        MANUAL, WAIT, AUTOSCORE
    };
    
    public Mode mode;
    public int mPhase;
    public double mPhaseNextTime = Double.MAX_VALUE;
    public int mPhaseNext;
    public ElapsedTime mTimer;
    public ElapsedTime hookTimer = null;
    public ElapsedTime loadTimer = null;

    public int retractState = 0;
    public int retractLtiltAt = 200;
    public int retractRtiltAt = 200;
    public double retractTiltVelocity = TILT_RETRACT_VELOCITY;

    public int elevScoreLevel = 1;
    public boolean leftIntakePixel = false;
    public boolean rightIntakePixel = false;
    public boolean anyIntakePixel = false;
    public int intakeLevel = 0;
    public int scoreSide = 3;
    public boolean matrixChange = true;
    public double spivPos = SPIV_SETUP;
    public boolean spivMp = false; 
    
    public void init(HardwareMap hwMap){
        larm = hwMap.get(Servo.class,"larm");
        rarm = hwMap.get(Servo.class,"rarm");
        larm.setDirection(Servo.Direction.FORWARD);
        rarm.setDirection(Servo.Direction.REVERSE);

        intake = hwMap.get(DcMotorEx.class,"intake");

        gantry = hwMap.get(Servo.class, "gantry");
        lscore = hwMap.get(Servo.class, "lscore");
        rscore = hwMap.get(Servo.class,"rscore");
        lscore.setDirection(Servo.Direction.FORWARD);
        rscore.setDirection(Servo.Direction.REVERSE);
        spiv = hwMap.get(Servo.class, "spiv");
        rhook = hwMap.get(Servo.class,"rhook");
        lhook = hwMap.get(Servo.class,"lhook");
        rhook.setDirection(Servo.Direction.FORWARD);
        lhook.setDirection(Servo.Direction.REVERSE);

        ltilt = hwMap.get(DcMotorEx.class, "ltilt");
        rtilt = hwMap.get(DcMotorEx.class, "rtilt");
        ltilt.setTargetPositionTolerance(20);
        rtilt.setTargetPositionTolerance(20);
        ltilt.setPower(0);
        rtilt.setPower(0);
        //ltilt.setVelocityPIDFCoefficients(10, 1, 0, 0);
        //rtilt.setVelocityPIDFCoefficients(10, 1, 0, 0);
        elev = hwMap.get(DcMotorEx.class, "elev");
        elev.setTargetPositionTolerance(20);
        elev.setPower(0);
        elev.setDirection(DcMotor.Direction.REVERSE);
        elev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elev.setVelocityPIDFCoefficients(10, 3, 0, 13);
        elim = hwMap.get(DigitalChannel.class, "elim");
        ltlim = hwMap.get(DigitalChannel.class, "ltlim");
        rtlim = hwMap.get(DigitalChannel.class, "rtlim");
        lcolor = hwMap.get(NormalizedColorSensor.class, "lcolor");
        rcolor = hwMap.get(NormalizedColorSensor.class, "rcolor");
        ldistance = (DistanceSensor) lcolor;
        rdistance = (DistanceSensor) rcolor;
        lball = hwMap.get(DigitalChannel.class, "lball");
        rball = hwMap.get(DigitalChannel.class, "rball");
        
        cLed = hwMap.get(Blinker.class, "Control Hub");
        eLed = hwMap.get(Blinker.class, "Expansion Hub 2");
        
        matrix = hwMap.get(AdafruitLEDBackpack.class, "matrix");
        matrixChange = true;
        
        launcher = hwMap.get(Servo.class,"launcher");

        mTimer = new ElapsedTime();
        hookTimer = null;
        setMode(Mode.IDLE);
    }

    public void setMode(Mode m){
        if(mode != m){
            resetMode(m);
        }
    }

    public void setScoreSide(int side) {
        scoreSide = side;
    }

    //force mode change
    public void resetMode(Mode m) {
        mode = m;
        mPhase = 0;
        mTimer.reset();
        matrixChange = true;
    }

    public boolean isMode(Mode m){
        return mode == m;
    }
    
    public boolean isBusy() {
        if (mode == Mode.WAIT) return false;
        if (mode == Mode.IDLE) return false;
        if (mode == Mode.STOW) return false;
        if (mode == Mode.EXTENDED) return false;
        return true;
    }

    public void modePause(double s){
        mPhaseNextTime = mTimer.seconds() + s;
        mPhaseNext = mPhase + 1;
        mPhase = -1;
    }

    public void update(){
        leftIntakePixel = (ldistance.getDistance(DistanceUnit.CM) < 1.5);
        rightIntakePixel = (rdistance.getDistance(DistanceUnit.CM) < 2);
        anyIntakePixel = anyIntakePixel || leftIntakePixel || rightIntakePixel;
        if(mTimer.seconds() > mPhaseNextTime){
            mPhase = mPhaseNext;
            mPhaseNextTime = Double.MAX_VALUE;
        }
        if(mode == Mode.IDLE) doIDLE();
        if(mode == Mode.STOW) doSTOW();
        if(mode == Mode.SETUP) doSETUP();
        if(mode == Mode.INTAKE) doINTAKE();
        if(mode == Mode.OUTTAKE) doOUTTAKE();
        if(mode == Mode.INTAKEIDLE) doINTAKEIDLE();
        if(mode == Mode.HOLD) doHOLD();
        if(mode == Mode.XFER) doXFER();
        if(mode == Mode.SWING) doSWING();
        if(mode == Mode.EXTEND) doEXTEND();
        if(mode == Mode.SCORE) doSCORE();
        if(mode == Mode.PURPLE) doPURPLE();
        if(mode == Mode.RETRACT) doRETRACT();
        if(mode == Mode.HANG) doHANG();
        if(mode == Mode.LAUNCH) doLAUNCH();
        if(mode == Mode.CLIMB) doCLIMB();
        if(mode == Mode.AUTOSCORE) doAUTOSCORE();

        updateRetract();
        if (matrixChange) updateMatrix();
        if (hookTimer != null && hookTimer.seconds() > 5) {
            hookDisable();
        }
    }
    
    public void updateMatrix() {
        matrix.clear();
        matrix.drawString(elevScoreLevel+"", 0, 0, matrix.LED_AMBER, 0);
        if (anyIntakePixel)
            matrix.drawBitmap(-1L, 3, 6, matrix.LED_GREEN, 0, 2, 2);
        if (leftIntakePixel)
            matrix.drawBitmap(-1L, 0, 5, matrix.LED_GREEN, 0, 3, 3);
        if (rightIntakePixel)
            matrix.drawBitmap(-1L, 5, 5, matrix.LED_GREEN, 0, 3, 3);
        matrix.writeDisplay();
        matrixChange = false;
    }

    public void doIDLE() {
        if (mPhase == 0) {
            intake.setPower(0);
            mPhase++;
        }
    }
    
    public void doINTAKEIDLE() {
        if (mPhase == 0) {
            intake.setPower(0);
            mPhase++;
        }
        if (leftIntakePixel && rightIntakePixel) {
            setMode(Mode.XFER);
        }
    }
    
    public void doSETUP() {
        prepareLoad(false);
        if(mPhase == 0){
            anyIntakePixel = false;
            intake.setPower(0);
            grab(3);
            hookDown();
            gantry.setPosition(GANTRY_SETUP);
            larm.setPosition(LARM_SETUP);
            rarm.setPosition(RARM_SETUP);
            modePause(1);
        }
        if (mPhase == 1) {
            startRetract();
            mPhase++;
        }
        if (mPhase == 2 && retractState == 0) {
            hookDisable();
            setElevator(11);
            setMode(mode.IDLE);
        }
    }

    public void doINTAKE() {
        matrixChange = true;
        if (mPhase == 0) {
            intake.setPower(INTAKE_IN);
            if (intakeLevel == 3) {
                larm.setPosition(LARM_LEV3);
                rarm.setPosition(RARM_LEV3);
            } else if (intakeLevel == 4) {
                larm.setPosition(LARM_LEV4);
                rarm.setPosition(RARM_LEV4);
            } else {
                larm.setPosition(LARM_LEV0);
                rarm.setPosition(RARM_LEV0);
            }
            prepareLoad(true);
            mPhase++;
        }
        if (mPhase == 1) {
            if (leftIntakePixel && rightIntakePixel) {
                setMode(Mode.XFER);
            }
        }
    }
    
    public void doOUTTAKE() {
        anyIntakePixel = false;
        matrixChange = true;
        if (mPhase == 0) { 
            intake.setPower(INTAKE_OUT);
            mPhase++;
        }
    }

    public void doSTOW(){
        anyIntakePixel = false;
        prepareLoad(false);
        if (mPhase == 0) {
            intake.setPower(0);
            startRetract();
            larm.setPosition(LARM_STOW);
            rarm.setPosition(RARM_STOW);
            gantry.setPosition(GANTRY_STOW);
            lhook.setPosition(LHOOK_DOWN);
            rhook.setPosition(RHOOK_DOWN);
            mPhase++;
        }
        if (mPhase == 1) {
            if (retractState == 0 || mTimer.seconds() > 4) {
                setMode(mode.IDLE);
            }
        }
    }
    
    public void doHOLD(){
        if(mPhase == 0) {
            larm.setPosition(LARM_HOLD);
            rarm.setPosition(RARM_HOLD);
            intake.setPower(INTAKE_HOLD);
            mPhase++;            
        }
    }

    public void doXFER(){
        anyIntakePixel = false;
        matrixChange = true;
        if (mPhase == 0) {
            // prepare everything for transfer
            setElevator(12);
            prepareLoad(true);
            if (loadTimer.seconds() < 0.1) {
                larm.setPosition(LARM_HOLD);
                rarm.setPosition(RARM_HOLD);
                intake.setPower(INTAKE_HOLD);
            }   
            mPhase++;
        }
        if (mPhase == 1 && loadTimer.seconds() >= 0.1) {
            // start the transfer process
            intake.setPower(INTAKE_XFER);
            larm.setPosition(LARM_XFER);
            rarm.setPosition(RARM_XFER);
            modePause(1.2);
        }
        if (mPhase == 2) {
            // grab the pixels
            prepareLoad(false);
            lscore.setPosition(LSCORE_CLOSE);
            rscore.setPosition(RSCORE_CLOSE);
            modePause(0.25);
        }
        if(mPhase == 3){
            setMode(Mode.SWING);
        }
    }

    public void doSWING() {
        prepareLoad(false);
        if(mPhase == 0){
            intake.setPower(0);
            gantry.setPosition(GANTRY_SWING);
            modePause(0.1);
        }
        if(mPhase == 1){
            spiv.setPosition(SPIV_SWING);
            modePause(0);
        }
        if(mPhase == 2){
            setMode(Mode.WAIT);
        }
    }

    public void doEXTEND(){
        prepareLoad(false);
        if(mPhase == 0){
            gantry.setPosition(GANTRY_SCORE);
            setElevator(elevScoreLevel);
            modePause(0);
        }
        if(mPhase == 1){
            if (!elev.isBusy() && !ltilt.isBusy() && !rtilt.isBusy())
                setMode(Mode.EXTENDED);
                return;
        }
    }

    public void doSCORE(){
        prepareLoad(false);
        if(mPhase == 0){
            if (scoreSide == 1) { 
                lscore.setPosition(LSCORE_OPEN);
            } else if (scoreSide == 2) {
                rscore.setPosition(RSCORE_OPEN);
            } else {
                lscore.setPosition(LSCORE_OPEN);
                rscore.setPosition(RSCORE_OPEN);
            }
            modePause(0.15);
        }
        if(mPhase == 1){
            if(scoreSide == 1 || scoreSide == 2){
                setMode(mode.WAIT);
                return;
            }
            spiv.setPosition(SPIV_RETRACT);
            ltilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ltilt.setTargetPosition(ltilt.getCurrentPosition()+250);
            ltilt.setVelocity(TILT_VELOCITY);
            rtilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rtilt.setTargetPosition(rtilt.getCurrentPosition()+250);
            rtilt.setVelocity(TILT_VELOCITY);
            modePause(0.5);
        }
        if(mPhase == 2){
            if (scoreSide == 1 || scoreSide == 2) setMode(Mode.WAIT);
            else setMode(Mode.RETRACT);
        }
    }
    
    public void doPURPLE() {
        prepareLoad(false);
        if(mPhase == 0){
            rscore.setPosition(RSCORE_OPEN);
            modePause(0.15);
        }
        if(mPhase == 1) {
            spiv.setPosition(SPIV_SCORE);
            modePause(0.1);
        }
        if(mPhase == 2) {
            setMode(Mode.WAIT);
            return;
        }
        
    }

    public void doRETRACT(){
        prepareLoad(false);
        if (mPhase == 0) {
            startRetract();
            gantry.setPosition(GANTRY_STOW);
            spiv.setPosition(SPIV_RETRACT);
            modePause(0.5);
        }
        if (mPhase == 1) {
            setMode(Mode.WAIT);
        }
    }
    
    public void doHANG() {
        prepareLoad(false);
        if (mPhase == 0) {
            larm.setPosition(LARM_LEV3);
            rarm.setPosition(RARM_LEV3);
            setElevator(13);
            modePause(0.5);
        }
    }

    public void doLAUNCH() {
        prepareLoad(false);
        if (mPhase == 0) {
           launch(true);
           modePause(0.4);
        }
        if (mPhase == 1) {
           hookUp();
           mPhase++;
        }
    }
    
    public void doCLIMB() {
        prepareLoad(false);
        if (mPhase == 0) {
            //startRetract(TILT_CLIMB_VELOCITY);
            setElevator(23, TILT_CLIMB_VELOCITY);
            mPhase++;
        }
        if (mPhase == 1) {
            setMode(Mode.IDLE);
        }
    }
    
    public void prepareLoad(boolean enable) {
        if (!enable) loadTimer = null;
        if (enable && loadTimer == null) {
            setElevator(12);
            lscore.setPosition(LSCORE_OPEN);
            rscore.setPosition(RSCORE_OPEN);
            gantry.setPosition(GANTRY_XFER);
            spiv.setPosition(SPIV_XFER);
            loadTimer = new ElapsedTime();
        }
    }
    
    public void doAUTOSCORE(){
        prepareLoad(false);
        if(mPhase == 0){
            if (scoreSide == 1) { 
                lscore.setPosition(LSCORE_OPEN);
            } else if (scoreSide == 2) {
                rscore.setPosition(RSCORE_OPEN);
            } else {
                lscore.setPosition(LSCORE_OPEN);
                rscore.setPosition(RSCORE_OPEN);
            }
            modePause(0.15);
        }
        if(mPhase == 1){
            if(scoreSide == 1 || scoreSide == 2){
                setMode(mode.WAIT);
                return;
            }
            spiv.setPosition(SPIV_RETRACT);
            mPhase++; 
        }
        if(mPhase == 2){
            if (scoreSide == 1 || scoreSide == 2) setMode(Mode.WAIT);
            else setMode(Mode.RETRACT);
        }
        
    }
    
    public void startRetract() {
        startRetract(TILT_RETRACT_VELOCITY);
    }
    public void startRetract(double tiltvel) {
        prepareLoad(false);
        retractTiltVelocity = tiltvel;
        retractState = 31;
    }

    public void cancelRetract() {
        retractState = 0;
    }

    public void updateRetract() {
        if (retractState == 0) return;
        boolean ezero = !elim.getState();
        boolean ltzero = !ltlim.getState();
        boolean rtzero = !rtlim.getState();
        int elevPos = elev.getCurrentPosition();
        if (retractState == 8) {
            setElevator(0);
            retractState = 0;
        }
        if ((retractState & 16) == 16) {
            // start motors if needed
            if (!ezero) {
                retractLtiltAt = Math.max(elevPos - 50, 200);
                retractRtiltAt = retractLtiltAt;
                elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elev.setVelocity(ELEV_RETRACT_VELOCITY);
            }
            if (!ltzero) {
                ltilt.setVelocity(0);  // wait to reach retractTiltAt
                ltilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (!rtzero) {
                rtilt.setVelocity(0);  // wait to reach retractTiltAt
                rtilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            retractState -= 16;
        }
        if (((retractState & 1) == 1) && ezero) {
            elev.setVelocity(0);
            elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elev.setPower(0);
            elevPos = 0;
            retractState -= 1;
        }
        if ((retractState & 2) == 2) {
            if (ltzero) { 
                ltilt.setVelocity(0);
                ltilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                retractState -= 2;
            } else if (elevPos < retractLtiltAt) {
                ltilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ltilt.setVelocity(retractTiltVelocity);
                retractLtiltAt = -9999;
            }
        }
        if ((retractState & 4) == 4) {
            if (rtzero) {
                rtilt.setVelocity(0);
                rtilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                retractState -= 4;
            } else if (elevPos < retractRtiltAt) {
                rtilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rtilt.setVelocity(retractTiltVelocity);
                retractRtiltAt = -9999;
            }
        }
    }
    
    public void setElevator(int p) { setElevator(p, TILT_DOWN_VELOCITY); }

    public void setElevator(int p, double retractVelocity) {
        cancelRetract();
        double[] pos = elevPos[p];
        spivPos = pos[2];

        ltilt.setTargetPosition((int) pos[0]);
        rtilt.setTargetPosition((int) pos[0]);
        elev.setTargetPosition((int) pos[1]);
        spiv.setPosition(spivPos);
        ltilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rtilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (ltilt.getCurrentPosition() > ltilt.getTargetPosition()) {
            ltilt.setVelocity(retractVelocity);
            rtilt.setVelocity(retractVelocity);
        } else {
            ltilt.setVelocity(TILT_VELOCITY);
            rtilt.setVelocity(TILT_VELOCITY);
        }
        elev.setVelocity(ELEV_VELOCITY);
        matrixChange = true;
    }

    public void adjustElev(double vel) {
        prepareLoad(false);
        cancelRetract();
        setMode(Mode.MANUAL);
        if (vel != 0) {
            elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elev.setVelocity(vel);
        }
        if (vel == 0 && elev.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            elev.setTargetPosition(elev.getCurrentPosition());
            elev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elev.setVelocity(ELEV_VELOCITY);
        }
    }
    
    public void adjustTilt(double vel) {
        prepareLoad(false);
        cancelRetract();
        double spivPos = Range.scale(ltilt.getCurrentPosition(), elevPos[1][0], elevPos[7][0], elevPos[1][2], elevPos[7][2]);
        if (!spivMp) { spiv.setPosition(spivPos); }
        setMode(Mode.MANUAL);
        ltilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ltilt.setVelocity(vel);
        rtilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rtilt.setVelocity(vel);
    }

    public void setLevel(int n) {
        matrixChange = true;
        if (n >= 1 && n < elevPos.length) {
            elevScoreLevel = n;
            if (!isMode(Mode.RETRACT)
                    && retractState == 0
                    && ltilt.getCurrentPosition() > 100
                    && rtilt.getCurrentPosition() > 100
                    && elev.getCurrentPosition() > 300) {
                resetMode(Mode.EXTEND);        
            }
        }
    }
    
    public int getLevel() {
        return elevScoreLevel;
    }
    
    public void adjustTeleopLevel(int n) {
        setLevel(Range.clip(getLevel()+n, TELEOP_LEVEL_MIN, TELEOP_LEVEL_MAX));
    }
    
    
    public int getBallSw() {
        int b = (lball.getState()) ? 0 : 1;
        b += (rball.getState()) ? 0 : 2;
        return b;
    }
    
    public void grab(int side) {
        lscore.setPosition((side & 1) == 1 ? LSCORE_CLOSE : LSCORE_OPEN);
        rscore.setPosition((side & 2) == 2 ? RSCORE_CLOSE : RSCORE_OPEN);
    }

    public void setLEDPattern(int a) {
        ArrayList<Blinker.Step> pat = new ArrayList<Blinker.Step>();
        int t = (a == 0) ? 250 : 1000;
        if (a == 0 || a == 1)
            pat.add(new Blinker.Step(0x00ffff, t, TimeUnit.MILLISECONDS));
        if (a == 0 || a == 2)
            pat.add(new Blinker.Step(0xff0000, t, TimeUnit.MILLISECONDS));
        if (a == 0) 
            pat.add(new Blinker.Step(0x00ff00, t, TimeUnit.MILLISECONDS));
        cLed.setPattern(pat);
        eLed.setPattern(pat);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("mode", "mode:%s phase:%d mtimer:%.2f", mode, mPhase, mTimer.seconds());
        telemetry.addData("score level", elevScoreLevel);
        telemetry.addData("retractState", retractState);
        telemetry.addData("hookpos", "lhook=%.3f rhook=%.3f", lhook.getPosition(), rhook.getPosition());
        telemetry.addData("elev", TelemetryEx.format(elev));
        telemetry.addData("ltilt", TelemetryEx.format(ltilt));
        telemetry.addData("rtilt", TelemetryEx.format(rtilt));
        telemetry.addData("limit", TelemetryEx.format(elim, "elim ")
                                    + TelemetryEx.format(ltlim, "ltlim ")
                                    + TelemetryEx.format(rtlim, "rtlim "));
        telemetry.addData("ldistance", ldistance.getDistance(DistanceUnit.CM));
        telemetry.addData("rdistance", rdistance.getDistance(DistanceUnit.CM));
        telemetry.addData("pixels in", "%s %s", leftIntakePixel, rightIntakePixel);
        telemetry.addData("ball sensors", getBallSw());
        telemetry.addLine("test");
    }
    
    public String csv(boolean header) {
        if (header) 
            return "mode,mphase,mtimer,elev,ltilt,rtilt";
        return 
            String.format("%s,%1d,%.1f,%d,%d,%d", mode, mPhase, mTimer.seconds(),
                elev.getCurrentPosition(),ltilt.getCurrentPosition(),rtilt.getCurrentPosition());
    }
    public String csv() { return csv(false); }
    
    public void addMotorTelemetry(Telemetry telemetry, String key, DcMotorEx m) {
        telemetry.addData(key, "%d->%d mode:%d %.2fA",
                            m.getCurrentPosition(), m.getTargetPosition(),
                            m.getMode().ordinal(), m.getCurrent(CurrentUnit.AMPS));
    }
    
    public void hookUp() {
        hookTimer = null;               // remove any existing hookTimer
        lhook.setPosition(LHOOK_UP);
        rhook.setPosition(RHOOK_UP);
    }
    
    public void hookDown() {
        hookTimer = new ElapsedTime();  // start a new hookTimer
        lhook.setPosition(LHOOK_DOWN);
        rhook.setPosition(RHOOK_DOWN);
    }
    
    public void hookDisable() {
        ((ServoImplEx)lhook).setPwmDisable();
        ((ServoImplEx)rhook).setPwmDisable();
        hookTimer = null;               // remove hookTimer
    }
    
    public void launch(boolean p){
        if(p) launcher.setPosition(LAUNCHER_OPEN); 
        else launcher.setPosition(LAUNCHER_CLOSE);
    }
    
    public int getElevPos(){
        return elev.getCurrentPosition();
    }
    
    public void setMpPos(){
        spivMp = !spivMp; 
        if (spivMp) {
            spiv.setPosition(SPIV_MP);
        }else{
            spiv.setPosition(SPIV_SCORE);
        }
    }
}

