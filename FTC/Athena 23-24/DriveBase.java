package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.RobotValues.*;


public class DriveBase {
    double HEADING_GAIN = 0.006;
    
    public static double defaultPoseLR = 6;
    public static double defaultPoseVel = 0;
    public static double defaultPoseH = 0;
    
    public DcMotorEx lf;
    public DcMotorEx lb;
    public DcMotorEx rb;
    public DcMotorEx rf;
    SparkFunOTOS myOtos;
    
    Pose field = new Pose();
    
    
    double fieldXOffset = 0;
    double fieldYOffset = 0;
    
    public void init(HardwareMap hwMap) {
        initMotors(hwMap);
        configureOtos(hwMap);
    }
    
    private void configureOtos(HardwareMap hwMap) {
        myOtos = hwMap.get(SparkFunOTOS.class, "otos");
        
        myOtos.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        myOtos.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-44.0/25.4, 148.0/25.4, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(0.995);
        myOtos.setAngularScalar(3600.0/3615.0);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);
    }
    
    public void updateOTOS() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        field.setCoords(pos.x + fieldXOffset, pos.y + fieldYOffset, pos.h);
    }
    
    public void initMotors(HardwareMap hwMap) {
        initMotors(hwMap, "lf", "lb", "rb", "rf");
    }
    
    public void initMotors(HardwareMap hwMap, String lfName, String lbName, String rbName, String rfName) {
        lf = initDriveMotor(hwMap, lfName, DcMotor.Direction.REVERSE);
        lb = initDriveMotor(hwMap, lbName, DcMotor.Direction.REVERSE);
        rb = initDriveMotor(hwMap, rbName, DcMotor.Direction.FORWARD);
        rf = initDriveMotor(hwMap, rfName, DcMotor.Direction.FORWARD);
        if (MOTOR_VELOCITY_PIDF != null) 
            setDriveVelocityPIDF(MOTOR_VELOCITY_PIDF);
    }
    
    public DcMotorEx initDriveMotor(HardwareMap hwMap, String name, DcMotor.Direction dir) {
        DcMotorEx m = hwMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return m;
    }
    
    public void setDriveMode(DcMotor.RunMode mode) {
        lf.setMode(mode);
        lb.setMode(mode);
        rb.setMode(mode);
        rf.setMode(mode);
    }

    public void setDriveVelocityPIDF(PIDFCoefficients pidf) {
        lf.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        lb.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rb.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rf.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
    
    public void driveXYW(double rx, double ry, double rw, double vel) {
        double denominator = Math.max(Math.abs(ry) + Math.abs(rx) + Math.abs(rw), 1);
        double lfPower = (rx - ry - rw) / denominator;
        double rfPower = (rx + ry + rw) / denominator;
        double lbPower = (rx + ry - rw) / denominator;
        double rbPower = (rx - ry +  rw) / denominator;

        if (lf.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            vel = vel * MAX_VELOCITY;
            lf.setVelocity(lfPower * vel);
            rf.setVelocity(rfPower * vel);
            lb.setVelocity(lbPower * vel);
            rb.setVelocity(rbPower * vel);
       } else {
            lf.setPower(lfPower * vel);
            rf.setPower(rfPower * vel);
            lb.setPower(lbPower * vel);
            rb.setPower(rbPower * vel);
        }
    }
    
    public double followPath(Path path) {
        return moveToPose(path.nextPose(field));
    }
    
    public double followPath(Path path, Ramp ramp) {
        return moveToPose(path.nextPose(field), ramp);
    }
    
    public double moveToPose(Pose pose) {
        return moveToPose(pose, null);
    }
    
    public double moveToPose(Pose pose, Ramp ramp) {
        double distRemain = pose.distRemain;
        if (pose.distRemain < 0) distRemain = 0;
        
        double vel = ramp == null ? pose.vel : ramp.scale(field.dist(pose) + distRemain);
        vel = Math.min(pose.vel, vel);
        return moveToXYH(pose.x, pose.y, pose.h, vel) + distRemain;
    }
    
    public double moveToXYHRamp(double tpx, double tpy, double th, double vel, Ramp ramp) {
        double rampVel = ramp.scale(distTo(tpx, tpy));
        return moveToXYH(tpx, tpy, th, Math.min(rampVel, vel));
    }
    
    public double moveToXYH(double tpx, double tpy, double th, double vel) {
        return moveToXYW(tpx, tpy, calcRVW(th), vel);
    }
    
    public double moveToXYW(double tpx, double tpy, double rvw, double vel) {
        double dist = distTo(tpx, tpy);
        double absHeadingRad = Math.atan2(tpy - field.y, tpx - field.x);
        double bearingRad = absHeadingRad - Math.toRadians(field.h);
        
        double rdx = Math.cos(bearingRad) * dist;
        double rdy = Math.sin(bearingRad) * dist;
        
        double dScale = Math.abs(rdx) + Math.abs(rdy);
        double rvx = rdx / dScale * vel;
        double rvy = rdy / dScale * vel * 2.5;
        
        driveXYW(rvx, rvy, rvw, 1);
        
        return dist;
    }
    
    public double calcRVW(double th) {
        return AngleUnit.normalizeDegrees(th - field.h) * HEADING_GAIN;
    }
    
    public double distTo(double tpx, double tpy) {
        return Math.hypot(tpx - field.x, tpy - field.y);
    }
    
    public void setPosition(double fpx, double fpy, double fph) {
        fieldXOffset = 0;
        fieldYOffset = 0;
        field.setCoords(fpx, fpy, fph);
        myOtos.setPosition(new SparkFunOTOS.Pose2D(fpx, fpy, fph));
    }
    
    public void stop() {
        driveXYW(0, 0, 0, 0);
    }
    
    public void setXYOffset(double x, double y) {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        fieldXOffset = x - pos.x;
        fieldYOffset = y - pos.y;
    }
    
    public boolean isHeading(double th, double tol) {
        return Math.abs(AngleUnit.normalizeDegrees(th-field.h)) < tol;
    }
    
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("pos", "x=%.2f y=%.2f h=%.2f", field.x, field.y, field.h);
    }

    public static class Pose {
        double y;       // y coordinate
        double x;       // x coordinate
        double h;       // heading (theta) angle
        double lr;      // segment lookahead radius
        double vel;     // relative velocity for this segment
        double distRemain = -1; // remaining distance to the end of the path
        
        public void setCoords(double x, double y, double h) {
            setValues(x, y, h, defaultPoseVel, defaultPoseLR);
        }
        
        public Pose() { 
            setValues(0,0,defaultPoseH,defaultPoseVel,defaultPoseLR); 
        }
        public Pose(double x, double y) { 
            setValues(x,y,defaultPoseH,defaultPoseVel,defaultPoseLR); 
        }
        public Pose(double x, double y, double h) { 
            setValues(x,y,h,defaultPoseVel,defaultPoseLR); 
        }
        public Pose(double x, double y, double h, double vel) {
            setValues(x,y,h,vel,defaultPoseLR);
        }
        public Pose(double x, double y, double h, double vel, double lr) {
            setValues(x,y,h,vel,lr);
        }
        public Pose setValues(double x, double y, double h, double vel, double lr) {
            this.x = x;
            this.y = y;
            this.h = h;
            this.vel = vel;
            this.lr = lr;
            return this;
        }

        public double dist(Pose t) { 
            return Math.hypot(t.y-y, t.x-x);
        }
        public double distToXY(double tx, double ty) {
            return Math.hypot(ty-y, tx-x);
        }
        public String toString() {
            return String.format("x=%.2f, y=%.2f, h=%.3f, distRemain=%.2f, lr=%.2f", x, y, h, distRemain, lr);
        }
    }
    
    public static class Path extends ArrayList<Pose> {
        public int current = 0;
        public String label = "";
        public int level = 0;
        public Path(String lab, int lev) { super(); label = lab; level = lev; }
        public Path(String lab) { super(); label = lab; level = 0; }
        public Path() { super(); }
        public Path addPose(Pose p) { add(p); return this; }
        public Path addPose(double x, double y)
            { add(new Pose(y,x,defaultPoseH,defaultPoseLR,defaultPoseVel)); return this; }
        public Path addPose(double x, double y, double h)
            { add(new Pose(x,y,h,defaultPoseLR,defaultPoseVel)); return this; }
        public Path addPose(double x, double y, double h, double vel)
            { add(new Pose(x,y,h,vel,defaultPoseLR)); return this; }
        public Path addPose(double x, double y, double h, double vel, double lr)
            { add(new Pose(x,y,h,vel,lr)); return this; }
            
        public Pose last() {
            if (size() < 1) return null;
            return get(size()-1);
        }
            
        public Pose nextPose(Pose from) {
            Pose p = get(current);
            while (current < size() - 1) {
                if (from.dist(p) > p.lr) {
                    break;
                }
                current++;
                p = get(current);
            }
            return p;
        }
        
        public void reset() {
            current = 0;
            double distance = 0;
            int i = size() - 1;
            if (i >= 0) {
                get(i).distRemain = 0;
                i--;
            }
            while (i >= 0) {
                distance += get(i).dist(get(i+1));
                get(i).distRemain = distance;
                i--;
            }
        }
    }
    
    public static class Ramp {
        double d0;
        double d1;
        double v0;
        double v1;
        
        public Ramp(double d0, double d1, double v0, double v1) {
            this.d0 = d0;
            this.d1 = d1;
            this.v0 = v0;
            this.v1 = v1;
            if (v0 > v1) {
                this.d0 = d1;
                this.d1 = d0;
                this.v1 = v0;
                this.v0 = v1;
            }
        }
        
        public double scale(double dist) {
            double vel = Range.scale(dist, d0, d1, v0, v1);
            vel = Range.clip(vel, v0, v1);
            return vel;
        }
    }
    
    public String csv(boolean header) {
        if (header) 
            return "fx,fy,fh";
        return 
            String.format(" %.1f,%.1f,%.3f",
                field.x, field.y, field.h
            );
    }
}
