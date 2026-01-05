package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.PwmControl;
import java.util.List;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.RobotValues.*;


public class DriveBaseCS extends DriveBase {
    
    private VisionPortal[] visionPortal = new VisionPortal[2];
    private AprilTagProcessor[] aprilTagProc = new AprilTagProcessor[2];
    public TeamPropProcessor propProc;
    public int updates = 0;
    public ElapsedTime tagTimer = null;
    public ElapsedTime aprilSeen = null;
    public DigitalChannel redLed = null;
    public DigitalChannel greenLed = null;
    public HuskyLens husky = null;
    public AnalogInput ultrasonic = null;
    public ServoImplEx fspot =  null;
    public SimpleLogger logfile;

    String idSeen = "";
    int idCount = 0;
    HuskyLens.Block foundBlock = null;
    
    public void initCamera(HardwareMap hwMap) {
        husky = hwMap.get(HuskyLens.class, "husky");
        ultrasonic = hwMap.get(AnalogInput.class, "usdist");
        fspot = hwMap.get(ServoImplEx.class, "fspot");
        fspot.setPwmRange(new PwmControl.PwmRange(0,19900));
        fspot.setPosition(0);
        greenLed = hwMap.get(DigitalChannel.class, "ledg");
        redLed = hwMap.get(DigitalChannel.class, "ledr");
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);
        redLed.setMode(DigitalChannel.Mode.OUTPUT);
        redLed.setState(true);
        greenLed.setState(true);
        aprilSeen = new ElapsedTime();
        // int[] viewIDs = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        
        aprilTagProc[0] = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProc[0].setDecimation(2);
        propProc = new TeamPropProcessor();
        
        visionPortal[0] = new VisionPortal.Builder()
            .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new android.util.Size(640, 360))
            .addProcessor(aprilTagProc[0])
            // .setLiveViewContainerId(viewIDs[0])
            .addProcessor(propProc)
            // .enableCameraMonitoring(false)
            .build();
        // aprilTagProc[1] = AprilTagProcessor.easyCreateWithDefaults();
        // aprilTagProc[1].setDecimation(2);
        // visionPortal[1] = new VisionPortal.Builder()
        //     .setCamera(hwMap.get(WebcamName.class, "Webcam 2"))
        //     .setCameraResolution(new android.util.Size(640, 360))
        //     .setLiveViewContainerId(viewIDs[1])
        //     .addProcessor(aprilTagProc[1])
        //     // .enableCameraMonitoring(false)
        //     .build();
    }
    
    public void setAlliance(int a, int b) { propProc.setAlliance(a, b); }
    public int getPropPos() { return propProc.getPropPos(); }
    public void propEnable(boolean enable) {
        visionPortal[0].setProcessorEnabled(propProc, enable);
    }
    
    public void updateAprilTagPosition(Telemetry telemetry) {
        double camera_y = CAMERA_Y[0];
        double camera_w = CAMERA_W[0];
        idCount = 0;
        List<AprilTagDetection> currentDetections = aprilTagProc[0].getDetections();
        
        if (currentDetections != null) {
            for (AprilTagDetection det : currentDetections) {
                if (det.id < 1 || det.id > 6) continue;
                idCount++;
                if (det.metadata != null) {
                    double tagX = det.metadata.fieldPosition.get(0);
                    double tagY = det.metadata.fieldPosition.get(1);
                    double camR = det.ftcPose.range;
                    double camBRad = Math.toRadians(det.ftcPose.bearing); 
                    double fhRad = Math.toRadians(field.h + 180);
                    
                    double camX = tagX - Math.cos(fhRad + camBRad) * camR;
                    double camY = tagY - Math.sin(fhRad + camBRad) * camR;
                    
                    double fpx = camX - Math.cos(fhRad) * 6;
                    double fpy = camY - Math.sin(fhRad) * 6;
                    
                    //setPosition(fpx, fpy, field.h);
                    setXYOffset(fpx, fpy);
                    
                    if (telemetry != null) {
                        // telemetry.addLine(String.format("\n==== (ID %d) %s", det.id, det.metadata.name));
                        telemetry.addLine(String.format("tag: id=%d fpxy=%6.1f,%6.1f", det.id, fpx, fpy));
                        // telemetry.addLine(String.format("%s", det.metadata.fieldPosition.toString()));
                        // telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        // telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        // telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", det.ftcPose.range, det.ftcPose.bearing, det.ftcPose.elevation));
                        // telemetry.addLine(String.format("tX tY %6.1f %6.1f", tagX, tagY));
                    }
                }
            }
        boolean ledstate = (idCount < 1);
        if (redLed.getState() != ledstate) redLed.setState(ledstate);
        }
    }
    
    public double[] getTagValues(double tag) {
        List<AprilTagDetection> currentDetections = aprilTagProc[0].getDetections();
        
        if (currentDetections != null) {
            for (AprilTagDetection det : currentDetections) {
                if (det.metadata != null) {
                    if (det.id == tag) {
                        return new double[] {det.ftcPose.range, det.ftcPose.bearing};
                    }
                }
            }
        }
        return null;
    }
    
    public HuskyLens.Block getHusky() {
        HuskyLens.Block[] blocks = husky.blocks();
        foundBlock = null;
        int foundX = 999;
        for (HuskyLens.Block b : blocks) {
            if (b.width <= 30) continue;       // skip narrow blocks
            if(b.y >= 200) continue;            //skip anything far from center
            int x = b.left+b.width/2;
            if (Math.abs(160-x) < foundX) {
                foundBlock = b;
                foundX = x;
            }
        }
        return foundBlock;
    }
    
    public double getUSDist() {
        double dist = Range.scale(ultrasonic.getVoltage(), 0.15, 0.207, 5.5, 9.25);
        return dist;
    }
    
    public void setFSpot(boolean on) {
        fspot.setPosition(on ? 1.0 : 0.0);
    }
    
    public String csv(boolean header) {
        if (header) 
            return super.csv(true)+" ,idCount,idSeen,huskyW,huskyL";
        int l = 0;
        int w = 0;
        if (foundBlock != null) { l = foundBlock.left; w = foundBlock.width; } 
        return super.csv(false) + String.format(" ,%d,\"%s\",%d,%d", idCount, idSeen, w, l);    
    }
    
    public String csv() { return csv(false); }
}