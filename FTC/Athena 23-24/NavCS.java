package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.RobotValues.*;

public class NavCS extends Nav {

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
        greenLed = hwMap.get(DigitalChannel.class, "ledg");
        redLed = hwMap.get(DigitalChannel.class, "ledr");
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);
        redLed.setMode(DigitalChannel.Mode.OUTPUT);
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
    
    public void setFSpot(boolean on) {
        fspot.setPosition(on ? 1.0 : 0.0);
    }
    
    public void updateAprilTagPosition() { updateAprilTagPosition(0, null); }
    public void updateAprilTagPosition(int camera) { updateAprilTagPosition(camera, null); }
    public void updateAprilTagPosition(int camera, Telemetry telemetry) {
        idSeen = ""; idCount = 0;
        if (camera < 0 || camera >= aprilTagProc.length) return;
        if (aprilTagProc[camera] == null) return;
        double camera_y = CAMERA_Y[camera];
        double camera_w = CAMERA_W[camera];
        List<AprilTagDetection> currentDetections = aprilTagProc[camera].getDetections();
        if (currentDetections != null) {
            double heading = getHeading();
            double cY = -Math.cos(heading*ANGTORAD) * camera_y;
            double cX = Math.sin(heading*ANGTORAD) * camera_y;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    idSeen = idSeen + detection.id; idCount++;
                    if (detection.id >= 1 && detection.id <= 6) {
                        aprilSeen.reset();
                        double tagY = detection.metadata.fieldPosition.get(1);
                        double tagX = detection.metadata.fieldPosition.get(0);
                        double tagR = detection.ftcPose.range;
                        double tagB = detection.ftcPose.bearing;
                        double theta = normalizeAngle(heading + (tagB/360.0) + camera_w + CIRCLE/2);
                        double fY = Math.cos(theta*ANGTORAD)*tagR;
                        double fX = -Math.sin(theta*ANGTORAD)*tagR;
                        field.y = tagY + fY + cY;
                        field.x = tagX + fX + cX;
                        updates++;
                        if (telemetry != null) {
                            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                            telemetry.addLine(String.format("%s", detection.metadata.fieldPosition.toString()));
                            // telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                            // telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                            telemetry.addLine(String.format("tY tX theta %6.1f %6.1f %6.1f", tagY, tagX, theta*360));
                            telemetry.addLine(String.format("fYX cYX %6.1f,%6.1f %6.1f,%6.1f ", fY, fX, cY, cX));
                            telemetry.addLine(String.format("YX %6.1f %6.1f", field.y, field.x));
                            telemetry.addLine(String.format("Dist: %6.1f", getUSDist()));
                        }
                    }
                }
            }
        }
        boolean ledon = !(aprilSeen.seconds() < 0.1);
        if (greenLed.getState() != ledon) greenLed.setState(ledon);
        if (!redLed.getState()) redLed.setState(true);
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

    public boolean setManualExposure(LinearOpMode op, int portal, int exposureMS, int gain) {
        if (portal < 0 || portal >= visionPortal.length) return false;
        VisionPortal vp = visionPortal[portal];
        if (vp == null) return false;
        while (!op.isStopRequested() && (vp.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            op.sleep(20);
        }
        if (!op.isStopRequested()) {
            ExposureControl exposureControl = vp.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                op.sleep(100);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            op.sleep(40);
    
            GainControl gainControl = vp.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            op.sleep(40);
        }
        return true;    
    }
    
    public String csv(boolean header) {
        if (header) 
            return super.csv(true)+" ,idCount,idSeen,huskyW,huskyL";
        int l = 0;
        int w = 0;
        if (foundBlock != null) { l = foundBlock.left; w = foundBlock.width; } 
        return super.csv(false) + String.format(" ,%d,\"%s\",%d,%d", idCount, idSeen, w, l);    
    } 
}
