package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

public class RobotValues { //0.24, 0.6
    public static final String SPECIMENPHASE = "/specimen/phase";
    public static final String SAMPLEPHASE = "/sample/phase";
    public static final String HANG = "/hang";
    public static final String HAS_SAMPLE = "/robot/hasSample";
    public static final String ROBOTTASK = "/robot/task";

    public static final int SLIDE_HORIZ_LIMIT = 1080;
    public static final int SLIDE_VERT_LIMIT = 2300;

    public static final double SWING_SETUP = 0.65;
    public static final double SWING_DOWN = 0.05;
    
    public static final int SLIDE_LIFT = 150; //150
    public static final int TILT_LIFT = 370;
    public static final double SWING_LIFT = SWING_DOWN;

    public static final int SLIDE_OBSNORTH = 56; //56
    public static final int TILT_OBSNORTH = 1400; //1318
    public static final double SWING_OBSNORTH = 0.50; //.55

    public static final int SLIDE_OBSWEST = 50; // (int)(0.85*SLIDE_HORIZ_LIMIT)
    public static final int TILT_OBSWEST = TILT_LIFT+80; //1318
    public static final double SWING_OBSWEST = SWING_DOWN; //.55

    public static final int SLIDE_BACK = 1400;//1400
    public static final int SLIDE_BACK_LOW = 850;
    public static final int TILT_BACK = 1560;
    public static final double SWING_BACK = 1;
    
    public static final int SLIDE_FRONT = 1390;
    public static final int TILT_FRONT = 1500;
    public static final double SWING_FRONT = 0.73;
    public static final double SWING_RETRACT = SWING_LIFT;
    
    public static final int SLIDE45 = 1300; //1430
    public static final int TILT45 = 1600; //1620
    public static final double SWING180 = 0.79;
    
    public static final int TILT_SEARCH = 315;
    public static final int TILT_SEARCH_MAX = 700;
    public static final int SLIDE_MAX = 1080;

    public static final int TILT_SWEEP = 190;
    public static final int SLIDE_SWEEP = 400;

    public static final double SPECGRAB_BACK = 0.720; //0.645
    public static final double SPECGRAB_FRONT = 0.720; //0.645
    public static final double SPECGRAB_CLOSE = 0.362; //0.30

    public static final double SPECWRIST_STOW = 0.043;
    public static final double SPECWRIST_INTAKE = 0.043; 
    public static final double SPECWRIST_HIGH = 0.780;
    public static final double SPECWRIST_DEGREE = (SPECWRIST_HIGH - SPECWRIST_INTAKE) / 180.0;
    
    public static final double SPECLATCH_UP = 0.980;
    public static final double SPECLATCH_DOWN = 0.25;
    
    public static final double SPECARM_STOW = 0.1;
    public static final double SPECARM_INTAKE = SPECARM_STOW;
    public static final double SPECARM_HIGH = 0.400;
    public static final double SPECARM_RETRACT = Range.scale(0.6, 0, 1, SPECARM_INTAKE, SPECARM_HIGH);
    public static final double SPECARM_SAVE = 0.5;

    public static final double PUSHBAR_OUT = .25;
    public static final double PUSHBAR_IN = .535;
    public static final double PUSHBAR_STOW = 0;
    public static final double PUSHBAR_PUSH = 0.857;
    public static final double PUSHBAR_READY = 0.343;
    
    public static final double TRAPDOOR_CLOSE = 0.36;
    public static final double TRAPDOOR_HOLD = 0.565;
    public static final double TRAPDOOR_SEARCH = 0.05; //0.36
    public static final double TRAPDOOR_INTAKE = 0.36;
    public static final double TRAPDOOR_BASKET = 0.05;
    public static final double TRAPDOOR_OBSZONE = 0.0;
    
    public static final double INTAKE_SPEED = 0;        // servo speed for intake
    public static final double INTAKE_STOP = 0.5;       // servo speed for stop/idle
    public static final double INTAKE_RELEASE = 0.63;    // servo speed for scoring
    public static final double INTAKE_RELEASE_OBS = 0.55;    // servo speed for obs zone
    public static final double INTAKE_RELEASE_SLOW = 0.58;    // servo speed for full basket
    public static final double INTAKE_REJECT = 0.9;     // servo speed for outtake/reject
    
    public static final double CLAW_CLOSE = 0.735; // 0.751
    public static final double CLAW_OPEN = 0.447;
    public static final double CLAW_SCORE = CLAW_OPEN;
    public static final double CLAW_WRIST[] = { 0.5,  0.67, 0.842, 0.33 };
    public static final double CLAW_WRIST_OBSNORTH = 0.71;

    public static final DriveBase.Ramp RAMP20 = new DriveBase.Ramp(36, 12, 0.2, 0.15);
    public static final DriveBase.Ramp RAMP40 = new DriveBase.Ramp(36, 6, 0.4, 0.25);
    public static final DriveBase.Ramp RAMP60 = new DriveBase.Ramp(12, 6, 0.6, 0.25);
    public static final DriveBase.Ramp RAMP70 = new DriveBase.Ramp(12, 6, 0.7, 0.25);
    public static final DriveBase.Ramp RAMP80 = new DriveBase.Ramp(12, 6, 0.8, 0.25);
    
    public static final double TAU = Math.PI * 2;
    public static final double CIRCLE = 1;
    public static final double ANGTORAD = TAU/CIRCLE;
    public static final double TICKS_PER_REVOLUTION = 384.5;
    public static final double GEAR_RATIO = 1.0 * 71.5/72.3;
    public static final double WHEEL_RADIUS = 48 / 25.4;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * TAU;
    public static final double WHEEL_TRACK = 149 / 25.4;
    public static final double WHEEL_BASE = 132 / 25.4;
    public static final double DISTANCE_PER_REVOLUTION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
    public static final double LATERAL_GAIN = 61.0/72.0;
    
    public static PIDFCoefficients MOTOR_VELOCITY_PIDF = new PIDFCoefficients(18,0,10,13);
    
    public static double GAIN_DRIVE_X = 1.1;
    public static double GAIN_HEADING = 4;
    public static double MAX_VELOCITY = 2600;
    
    public static final LogoFacingDirection LOGO_DIR = LogoFacingDirection.LEFT;
    public static final UsbFacingDirection USB_DIR = UsbFacingDirection.UP;
    

    public static final IMU.Parameters IMUPARAM = new IMU.Parameters(
         new RevHubOrientationOnRobot(
              new Orientation(
                   AxesReference.INTRINSIC,
                   AxesOrder.ZYX,
                   AngleUnit.DEGREES,
                   90,
                   0,
                   45,
                   0  // acquisitionTime, not used
              )
         )
    );
}