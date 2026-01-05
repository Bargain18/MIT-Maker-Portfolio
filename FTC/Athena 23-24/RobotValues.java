package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;

public class RobotValues {
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
    
    public static final double CAMERA_Y[] = { -172 / 25.4, 190 / 25.4 };
    public static final double CAMERA_X[] = { 0, 0 };
    public static final double CAMERA_W[] = { 0.5, 0 };


}