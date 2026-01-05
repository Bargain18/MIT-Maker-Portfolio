package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
import static org.firstinspires.ftc.teamcode.RobotValues.*;

public class ITDLocalize {
    public static final double CAMERA_X = -120 / 25.4;
    public static final double CAMERA_Y = 60 / 25.4;
    public static final double CAMERA_H = -90;

    private static VisionPortal visionPortal;
    private static AprilTagProcessor aprilTagProc;
    
    public static DigitalChannel aprilled;

    public static double fieldRotation = 0;
    public static double maxVelocity = 2;

    public static Telemetry telemetry = null;

    public static void initCamera(HardwareMap hwMap) {
        aprilTagProc = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProc.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
            .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new android.util.Size(640, 360))
            .addProcessor(aprilTagProc)
            .build();
            
        aprilled = hwMap.tryGet(DigitalChannel.class, "aprilled");
        if (aprilled != null) {
            aprilled.setMode(DigitalChannel.Mode.OUTPUT);
        }
    }
    
    public static double setFieldRotation(double fh) {
        fieldRotation = fh;
        return fieldRotation;
    }
    
    public static void updateAprilTagPosition(RobotHardware bot) {
        if (aprilTagProc == null) return;
        if (bot.getVelocity() > maxVelocity) return;
        boolean ledstate = true;  // aprilled off
        List<AprilTagDetection> currentDetections = aprilTagProc.getDetections();
        double headingrad = Math.toRadians(bot.getHeading());
        
        if (currentDetections != null) {
            for (AprilTagDetection det : currentDetections) {
                if (det.id != 12 && det.id != 15) continue;
                if (det.metadata != null) {
                    double bearing = det.ftcPose.bearing + CAMERA_H;
                    double range = det.ftcPose.range;
                    double tagX = det.metadata.fieldPosition.get(0);
                    double tagY = det.metadata.fieldPosition.get(1);
                    if (fieldRotation != 0) {
                        double theta = Math.toRadians(fieldRotation);
                        tagX = tagX * Math.cos(-theta) - tagY * Math.sin(-theta);
                        tagY = tagX * Math.sin(-theta) + tagY * Math.cos(-theta);
                    }                    
                    double theta = Math.toRadians(bearing) + headingrad;
                    double cfx = tagX - Math.cos(theta) * range;
                    double cfy = tagY - Math.sin(theta) * range;

                    double fx = cfx + CAMERA_X * Math.cos(-headingrad) - CAMERA_Y * Math.sin(-headingrad);
                    double fy = cfy + CAMERA_X * Math.sin(-headingrad) + CAMERA_Y * Math.cos(-headingrad);
                    
                    // if (fy > 90) {
                    //     bot.setAlliance(det.id < 14 ? 3 : 1);
                    //     continue;
                    // }
                    
                    bot.setPositionXY(fx, fy);
                    ledstate = false;  // aprilled on
                    // bot.setLED(0.500, 0.06);
                    
                    if (telemetry != null) {
                        telemetry.addLine(String.format("tag: id=%d range=%.1f fxy=%.1f,%.1f cxy=%.1f,%.1f", det.id, range, fx, fy, cfx, cfy));
                        telemetry.addLine(TelemetryEx.format(det));
                    }
                }
            }
        }
        if (aprilled != null && ledstate != aprilled.getState()) {
            aprilled.setState(ledstate);
        }
    }
}
