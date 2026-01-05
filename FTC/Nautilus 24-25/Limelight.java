package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.Collections;
import java.util.Comparator;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.Range;

public class Limelight {
    Limelight3A limelight;
    int pipeline = 7;
    String className = "red";
    
    ArrayList<LLDetection> validDets;
    LLResult result;
    public LLDetection targetPos;

    static double[][] tyToSlideMap = new double[][] {
            {-11.3, 8},
            {-6.34, 76},
            {-1.12, 111},
            {3.13, 222},
            {6.13, 353},
            {9.84, 485},
            {13.6, 723}
    };
    double minTy, maxTy;
    
    public static double limelightHeight = 9.85; // Camera height in inches
    public static double sampleHeight = 1.5;
    public static double limelightAngle = 60; // Camera angle (0° = down, 90° = forward)
    public static double clawLateralOffset = 5;
    int slideOffset = 50;
    
    public Limelight(HardwareMap hMap) {
        limelight = hMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(pipeline);
        limelight.start();

        Arrays.sort(tyToSlideMap, new Comparator<double[]>() {
            @Override
            public int compare(double[] a, double[] b) {
                return Double.compare(a[0], b[0]);
            }
        });
        minTy = tyToSlideMap[0][0];
        maxTy = tyToSlideMap[tyToSlideMap.length-1][0];
    }

    public int tyToSlideTicksLUT(double ty) {
        int i;
        for (i = 0; i < tyToSlideMap.length; i++) {
            if (tyToSlideMap[i][0] > ty) break;
        }
        double tyLow = tyToSlideMap[i-1][0], tyHigh = tyToSlideMap[i][0];
        double slideLow = tyToSlideMap[i-1][1], slideHigh = tyToSlideMap[i][1];
        return (int) Range.scale(ty, tyLow, tyHigh, slideLow, slideHigh);
    }
    
    public void update() {
        validDets = new ArrayList<>();
        result = limelight.getLatestResult();
        if (result == null) return;
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        if (detections.isEmpty()) return;
        
        for (LLResultTypes.DetectorResult detection : detections) {
            String name = detection.getClassName();
            if (!name.equals(className)) continue;
            
            double tx = detection.getTargetXDegrees();
            double ty = detection.getTargetYDegrees();
            
            if (ty < minTy || ty > maxTy) continue;
            
            double yAng = Math.toRadians(limelightAngle + ty);
            double xAng = Math.toRadians(tx);
            
            double yDist = (limelightHeight - sampleHeight) * Math.tan(yAng); 
            double xDist = Math.tan(xAng) * yDist;
            
            List<List<Double>> corners = detection.getTargetCorners();
            
            double dx = Math.toRadians(corners.get(1).get(0) - corners.get(0).get(0));
            double dy = Math.toRadians(corners.get(2).get(1) - corners.get(0).get(1));
            
            double horizontal = 0.842, vertical = 0.5;
            double angle = dx / dy > 2.5 ? vertical : horizontal;
            
            // if (yDist < 10) continue;
            double score = (xDist - clawLateralOffset) + (yDist - 10) * 1.5;
            validDets.add(new LLDetection(score, tx, ty, xDist - clawLateralOffset, tyToSlideTicksLUT(ty) + slideOffset, angle));
        }
    }
    
    public boolean processTargetPos() {
        targetPos = null;
        if (!validDets.isEmpty()) {
            Collections.sort(validDets, new Comparator<LLDetection>() {
                @Override
                public int compare(LLDetection a, LLDetection b) {
                    return Double.compare(a.score, b.score);
                }
            });
            targetPos = validDets.get(0);
        }
        return targetPos == null;
    }

    public void addTelemetry(Telemetry telemetry) {
        if (result == null) {
            telemetry.addLine("result is null");
            return;
        }
        if (result.getDetectorResults().isEmpty()) {
            telemetry.addLine("no detections found");
            return;
        }
        
        telemetry.addData("valid detections", validDets.size());
        
        for (LLDetection det : validDets) {
            telemetry.addData("tx", det.tx);
            telemetry.addData("ty", det.ty);
            telemetry.addData("xDist", det.xDist);
            telemetry.addData("targetSlide", det.slide);
            telemetry.addData("angle", det.angle);
            telemetry.addLine("----------------------");
        }
    }

    public void changeClassName(String className) {
        this.className = className;
    }

    public static class LLDetection {
        double score;
        double tx;
        double ty;
        double xDist;
        int slide;
        double angle;
        
        public LLDetection(double score, double tx, double ty, double xDist, int slide, double angle) {
            this.score = score;
            this.tx = tx;
            this.ty = ty;
            this.xDist = xDist;
            this.slide = slide;
            this.angle = angle;
        }
    }
}