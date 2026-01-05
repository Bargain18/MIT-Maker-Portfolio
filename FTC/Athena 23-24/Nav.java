package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import java.util.ArrayList;
import java.util.Arrays;
import static org.firstinspires.ftc.teamcode.RobotValues.*;

public class Nav {
    
    public static final double MOVEH_ABSFWD = 2000;
    public static final double MOVEH_ABSREV = 2001;
    public static final double MOVEH_RAMP = 2002;
    
    public static double defaultPoseH = MOVEH_ABSFWD;
    public static double defaultPoseLR = 12;
    public static double defaultPoseVel = 0.813;

    public static class Path extends ArrayList<Pose> {
        public Path addPose(Pose p) { add(p); return this; }
        public Path addPose(double y, double x)
            { add(new Pose(y,x,defaultPoseH,defaultPoseLR,defaultPoseVel)); return this; }
        public Path addPose(double y, double x, double h)
            { add(new Pose(y,x,h,defaultPoseLR,defaultPoseVel)); return this; }
        public Path addPose(double y, double x, double h, double lr)
            { add(new Pose(y,x,h,lr,defaultPoseVel)); return this; }
        public Path addPose(double y, double x, double h, double lr, double vel)
            { add(new Pose(y,x,h,lr,vel)); return this; }
    }

    public DcMotorEx lf;
    public DcMotorEx lb;
    public DcMotorEx rb;
    public DcMotorEx rf;
    public IMU imu;

    public Pose field = new Pose(0, 0, 0);
    
    public int lfTicksPrev = 0;
    public int lbTicksPrev = 0;
    public int rbTicksPrev = 0;
    public int rfTicksPrev = 0;
    
    public double rampD1 = 1;
    public double rampD0 = 0;
    public double rampV1 = 1;
    public double rampV0 = 0;
    public double moveVelMax = 1;
    public double moveVelMin = 0;
    
    public ArrayList<Pose> pathList = null;
    public double[] pathRemain =  null;
    public int pathCurrent = 0;
    public int pathTarget = 0;
    
    public double headingOffset = 0;
    public double headingPrev = 0;
    public long headingTime = 0;
    public long headingPrevTime = 0;

    public ElapsedTime countTimer = null;    
    public double countT = 0;
    public long trackCount = 0;
    public double trackPerSec = 0;
    public long headingCount = 0;
    public double headingPerSec = 0;
    
    double driveVY, driveVX, driveVW, driveVel;     // values from driveYXW()
    double errH;                                    // values from driveYXH()
    double moveX, moveY, moveVY, moveVX, moveH, moveVel;         // values from moveYXH()
    double pathY, pathX, pathH, pathVel, pathLR;
    double rdy, rdx, rdw;                       // values from updateTracking()
    double gdy, gdx, gdw;                       // values from updateTracking()
    
    public void init(HardwareMap hwMap) {
        countTimer = new ElapsedTime();
        initMotors(hwMap);
        initIMU(hwMap);
        setRampProfile(1,0,1,0);
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
    
    public void setDriveVelocityPIDF(double p, double i, double d, double f) {
        lf.setVelocityPIDFCoefficients(p, i, d, f);
        lb.setVelocityPIDFCoefficients(p, i, d, f);
        rb.setVelocityPIDFCoefficients(p, i, d, f);
        rf.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void initIMU(HardwareMap hwMap) {
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
            new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR));
        imu.initialize(params);
    }
    
    public double getIMUHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        headingTime = orientation.getAcquisitionTime();
        return orientation.getYaw(AngleUnit.RADIANS) / ANGTORAD;
    }
    public double getHeading() {
        return normalizeAngle(getIMUHeading() + headingOffset);
    }
    
    public boolean isHeading(double th, double tol) {
        return Math.abs(normalizeAngle(th-getHeading())) < tol;
    }
    
    public void resetIMUYaw() {
        imu.resetYaw();
    }    
    public void setYX(double y, double x) {
        updateTracking();
        field.y = y;
        field.x = x;
        field.h = getHeading();
    }
    public double setHeading(double heading) {
        double yawzero = getIMUHeading();
        headingOffset = heading - yawzero;
        field.h = heading;
        headingPrev = heading;
        return getHeading();
    }

    public void driveYXW(double ry, double rx, double rw, double vel) {
        rx = rx * GAIN_DRIVE_X;  // help correct strafing
        double denominator = Math.max(Math.abs(ry) + Math.abs(rx) + Math.abs(rw), 1);
        double lfPower = (ry + rx - rw) / denominator;
        double lbPower = (ry - rx - rw) / denominator;
        double rbPower = (ry + rx + rw) / denominator;
        double rfPower = (ry - rx + rw) / denominator;

        if (lf.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            vel = vel * MAX_VELOCITY;
            rf.setVelocity(rfPower * vel);
            rb.setVelocity(rbPower * vel);
            lf.setVelocity(lfPower * vel);
            lb.setVelocity(lbPower * vel);
       } else {
            rf.setPower(rfPower * vel);
            rb.setPower(rbPower * vel);
            lf.setPower(lfPower * vel);
            lb.setPower(lbPower * vel);
        }
        driveVY = ry; driveVX = rx; driveVW = rw; driveVel = vel;
    }
    
    public double driveYXH(double ry, double rx, double th, double vel) {
        errH = normalizeAngle(th - getHeading());
        double rw = Range.clip(errH * GAIN_HEADING, -.425, .425);
        driveYXW(ry * vel, rx * vel, rw, 1);
        return errH;
    }
    
    public void driveStop() { driveYXW(0,0,0,0); }
    
    public void updateTracking() { updateTracking(true); }
    public void updateTracking(boolean useIMU) {
        double heading = 0;
        double t = countTimer.seconds();
        double dT = t-countT;
        if (dT > 1.0) {
            trackPerSec = trackCount / dT;
            trackCount = 0;
            headingPerSec = headingCount / dT;
            headingCount = 0;
            countT = t;
        }
        trackCount++;

        // get current motor ticks
        int lfTicks = lf.getCurrentPosition();
        int lbTicks = lb.getCurrentPosition();
        int rbTicks = rb.getCurrentPosition();
        int rfTicks = rf.getCurrentPosition();

        // calculate wheel deltas as fraction of circle
        double lfD = (lfTicks-lfTicksPrev) / TICKS_PER_REVOLUTION;
        double lbD = (lbTicks-lbTicksPrev) / TICKS_PER_REVOLUTION;
        double rbD = (rbTicks-rbTicksPrev) / TICKS_PER_REVOLUTION;
        double rfD = (rfTicks-rfTicksPrev) / TICKS_PER_REVOLUTION;
        
        lfTicksPrev = lfTicks;
        lbTicksPrev = lbTicks;
        rbTicksPrev = rbTicks;
        rfTicksPrev = rfTicks;
        
        // calculate robot delta velocities in field units
        rdy = ((lfD + lbD + rbD + rfD) * DISTANCE_PER_REVOLUTION) / 4.0;
        rdx = ((lfD - lbD + rbD - rfD) * DISTANCE_PER_REVOLUTION) / 4.0 * LATERAL_GAIN;
        double w = ((rfD + rbD - lbD - lfD) * DISTANCE_PER_REVOLUTION)
                        / (4.0 * (WHEEL_TRACK + WHEEL_BASE));
        rdw = w / ANGTORAD;  // convert w in radians to rdw in angle units

        if (useIMU) {
            heading = getHeading();
            if (headingTime > headingPrevTime) {
                headingCount++;
                rdw = heading - headingPrev;
                w = rdw * ANGTORAD;
                headingPrev = heading; 
                headingPrevTime = headingTime;
            } else { useIMU = false; }
        }

        // calculate pose exponential
        double s;
        double c;
        if (w < -0.1 || w > 0.1) {
            s = Math.sin(w) / w;
            c = (1-Math.cos(w))/w;
        } else {
            // for angles near zero, approx sin()/cos() formulas using Taylor series
            // avoids div by zero, small denominator effects, and it's faster
            // (for range -0.1..+0.1, maximum error is less than 0.0025 degrees)
            s = 1 - ((w*w)/6);         // sin(w)/w      ~~>  (1-(w*w)/6)
            c = w/2;                   // (1-cos(w))/w  ~~>  (w/2)
        }
        double pdy = s * rdy - c * rdx;
        double pdx = c * rdy + s * rdx;
        double pdw = rdw;
        
        // compute delta change
        double w0 = field.h * ANGTORAD;
        gdy = Math.cos(w0)*pdy + Math.sin(w0)*pdx;
        gdx = Math.cos(w0)*pdx - Math.sin(w0)*pdy;
        gdw = pdw;
        
        // integrate into field coordinates
        field.y += gdy;
        field.x += gdx;
        field.h = (useIMU) ? heading : field.h + gdw;
    }
    
    public double distToYX(double y, double x) { return field.distToYX(y,x); }
    
    public void setRampProfile(double d1, double d0, double v1, double v0) {
        rampD1 = d1; rampD0 = d0; rampV1 = v1; rampV0 = v0;
    }

    public void moveRampDist(double dist) {
        moveVelMin = Math.min(rampV0, rampV1);
        moveVelMax = Range.clip(
                        Range.scale(dist, rampD1, rampD0, rampV1, rampV0),
                        moveVelMin, Math.max(rampV0, rampV1));
    }    
    
    public double moveToYXH(double ty, double tx, double th, double vel) {
        moveY = ty; moveX = tx;
        double dist = Math.hypot(ty-field.y, tx-field.x);
        double absHeading = (Math.atan2(ty-field.y, tx-field.x)-TAU/4)/ANGTORAD;
        double relHeading = normalizeAngle(absHeading-getHeading());
        double relY = Math.cos(relHeading * ANGTORAD) * dist;
        double relX = -Math.sin(relHeading * ANGTORAD) * dist;
        double dScale = Math.abs(relX) + Math.abs(relY);
        double vx = relX / dScale;
        double vy = relY / dScale;
        
        double endh = th % 1;
        double ramp;
        if (th>=2000) {
            switch ((int)th) {
                case 2000: th = absHeading; break;
                case 2001: th = absHeading - CIRCLE/2; break;
                case 2002:
                    th = absHeading;
                    while (th < endh - CIRCLE/4) th += CIRCLE/2;
                    while (th > endh + CIRCLE/4) th -= CIRCLE/2;
                    ramp = Math.max(Range.scale(dist - 12, 0, 12, 0, CIRCLE/4), 0);
                    th = Range.clip(th, endh-ramp, endh+ramp);
                    break;
                case 2003:
                    th = field.h;
                    while (th < endh - CIRCLE/2) th += CIRCLE;
                    while (th > endh + CIRCLE/2) th -= CIRCLE;
                    ramp = Math.max(Range.scale(dist - 4, 0, 12, 0, CIRCLE/4), 0);
                    th = Range.clip(th, endh-ramp, endh+ramp);
                    break;
            }
        }
        // handle any ramping limits here
        vel = Range.clip(vel, moveVelMin, moveVelMax);
        moveVY = vy; moveVX = vx; moveH = normalizeAngle(th); moveVel = vel;
        driveYXH(vy, vx, moveH, vel);
        return dist;
    }

    public void setPath(Path p) {
        pathList = p;
        int n = pathList.size();
        pathRemain = new double[n+1];
        for (int i = n-2; i>=0; i--) {
            pathRemain[i] = pathRemain[i+1] + pathList.get(i).dist(pathList.get(i+1));
        }
        pathCurrent = 0;
        pathTarget = 0;
    }
    
    public double getPathRemainingDist() {
        if (pathList == null) return 0;
        int n = pathList.size();
        int nextPose = pathCurrent;
        if (pathCurrent < n - 1) nextPose++;
        return field.dist(pathList.get(nextPose)) + pathRemain[nextPose];
    }
    
    public double followPath() {
        if (pathList == null) return -1;
        int n = pathList.size();
        if (pathCurrent >= n) { driveStop(); return pathCurrent; }
        Pose p0 = pathList.get(pathCurrent);
        Pose p1 = null;
        while (pathCurrent < n - 1) {
            p1 = pathList.get(pathCurrent + 1);
            if (field.dist(p1) > p0.lr) break;
            p0 = p1; p1 = null;
            pathCurrent++;
        }
        double distRemain = pathRemain[pathCurrent];
        pathY = p0.y;
        pathX = p0.x;
        pathH = p0.h;
        pathVel = p0.vel;
        pathLR = p0.lr;
        if (p1 != null) { pathY = p1.y; pathX = p1.x; }
        if (p1 != null && p0.lr > 0) {
            // Pure Pursuit movement calculation
            double x1 = p0.x - field.x;
            double y1 = p0.y - field.y;
            double x2 = p1.x - field.x;
            double y2 = p1.y - field.y;
            // if (Math.abs(x2-x1) < 0.003) x1 = x2 + 0.003;
            // if (Math.abs(y2-y1) < 0.003) y1 = y2 + 0.003;
            double dx = x2 - x1;
            double dy = y2 - y1;
            double dr = Math.hypot(dy, dx);
            double dr2 = dr * dr;
            double D = x1*y2 - x2*y1;
            double disc = p0.lr * p0.lr * dr2 - D * D;
            if (disc >= 0) {
                double sgn = (dy < 0) ? -1 : 1;
                double sqrtdisc = Math.sqrt(disc);
                double tx = (D * dy + sgn * dx * sqrtdisc) / dr2;
                double ty = (-D * dx + Math.abs(dy) * sqrtdisc) / dr2;
                double p0Dist = -1;
                if ( ((tx-x1)*(tx-x2)<=0) && ((ty-y1)*(ty-y2)<=0) ) {
                    pathY = ty + field.y;
                    pathX = tx + field.x;
                    p0Dist = p0.distToYX(pathY, pathX);
                }
                // try second intersection point
                tx = (D * dy - sgn * dx * sqrtdisc) / dr2;
                ty = (-D * dx - Math.abs(dy) * sqrtdisc) / dr2;
                double d2 = p0.distToYX(ty + field.y,tx + field.x);
                if (((tx-x1)*(tx-x2)<=0) && ((ty-y1)*(ty-y2)<=0) ) {
                    if (d2 > p0Dist) {
                        pathY = ty + field.y;
                        pathX = tx + field.x;
                        p0Dist = d2;
                    }
                }
            }
        }
        return distRemain + moveToYXH(pathY, pathX, pathH, pathVel);
    }
    
    public void addTelemetry(Telemetry t) {
        t.addData("nav.field", field.toString());
        t.addData("nav.trackPerSec", "%6.2f (%.1fms)", trackPerSec, 1000.0/trackPerSec);
        t.addData("pathCounter", pathCurrent);
        t.addData("path remaining dist", Arrays.toString(pathRemain));
        double h = getHeading();
        t.addData("heading", "%.3f(%.1f) @ %dns", h, h*360, headingTime);
        t.addData("lf", TelemetryEx.format(lf));
        t.addData("lb", TelemetryEx.format(lb));
        t.addData("rb", TelemetryEx.format(rb));
        t.addData("rf", TelemetryEx.format(rf));
    }
    
    public static double normalizeAngle(double a) {
        a = a % 1;
        while (a < -0.5) a += 1;
        while (a > 0.5) a -= CIRCLE;
        return a;
    }
    
    public String csv(boolean header) {
        if (header) 
            return " fy,fx,fh, pathCurrent,pathY,pathX,pathH,pathLR, moveY,moveX,moveH,moveVel,moveVelMax,moveVelMin,moveVY,moveVX, driveVY,driveVX,driveW,driveVel";
        return 
            String.format(" %.1f,%.1f,%.3f, %d,%.1f,%.1f,%.3f,%.1f, %.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f, %.2f,%.2f,%.2f,%.2f",
                        field.y, field.x, field.h,
                        pathCurrent,pathY,pathX,pathH,pathLR,
                        moveY,moveX,moveH,moveVel,moveVelMax,moveVelMin,moveVY,moveVX,
                        driveVY,driveVX,driveVW,driveVel
            );
    }
    public String csv() { return csv(false); }

    public static class Pose {
        double y;       // y coordinate
        double x;       // x coordinate
        double h;       // heading (theta) angle
        double lr;      // segment lookahead radius
        double vel;     // relative velocity for this segment
        
        public Pose() { 
            setValues(0,0,defaultPoseH,defaultPoseLR,defaultPoseVel); 
        }
        public Pose(double y, double x) { 
            setValues(y,x,defaultPoseH,defaultPoseLR,defaultPoseVel); 
        }
        public Pose(double y, double x, double h) { 
            setValues(y,x,h,defaultPoseLR,defaultPoseVel); 
        }
        public Pose(double y, double x, double h, double lr, double vel) {
            setValues(y,x,h,lr,vel);
        }
        public Pose setValues(double y, double x, double h, double lr, double vel) {
            this.y = y;
            this.x = x;
            this.h = h;
            this.lr = lr;
            this.vel = vel;
            return this;
        }

        public double dist(Pose t) { 
            return Math.hypot(t.y-y, t.x-x);
        }
        public double distToYX(double ty, double tx) {
            return Math.hypot(ty-y, tx-x);
        }
        public String toString() {
            return String.format("%.2f, %.2f, %.3f", y, x, h);
        }
    }

}

        

