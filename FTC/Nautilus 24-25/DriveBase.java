package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.function.Supplier;
import org.pmrobotics.ledmatrix.GoBildaPinpointDriver;

import static org.firstinspires.ftc.teamcode.RobotValues.*;


public class DriveBase {
    
    public Dispatcher dx;
    
    public static double defaultPoseLR = 6;
    public static double defaultPoseVel = 0;
    public static double defaultPoseH = 0;
    
    public DcMotorEx lf;
    public DcMotorEx lb;
    public DcMotorEx rf;
    public DcMotorEx rb;
    SparkFunOTOS myOtos;
    GoBildaPinpointDriver pinpoint;
    IMU chubIMU;

    double fieldXOffset = 0;
    double fieldYOffset = 0;
    
    public Pose field;
    public boolean disableDrive = false;
    
    public ElapsedTime atTargetTimer;
    
    public void init(HardwareMap hwMap) {
        dx = new Dispatcher();
        initMotors(hwMap);
        configurePinpoint(hwMap);
        atTargetTimer = new ElapsedTime();
        // configureOtos(hwMap);
        // initChubIMU(hwMap);
    }
    
    public void updateTracking() {
        updatePinpoint();
    }

    public double getHeading() { 
        return getPinpointHeading();
    }
    
    public double getVelocity() {
        return Math.hypot(pinpoint.getVelX(), pinpoint.getVelY()) / 25.4;
    }

    public void setPosition(double fpx, double fpy, double fph) {
        setPinpointPosition(fpx, fpy, fph);
    }
    
    public void setPositionXY(double fpx, double fpy) {
        Pose p = Pose.toPose(pinpoint.getPosition());
        fieldXOffset = fpx - p.x;
        fieldYOffset = fpy - p.y;
        field.x = fpx;
        field.y = fpy;
    }
    

    public double distTo(double tx, double ty) {
        return Math.hypot(tx - field.x, ty - field.y);
    }
    
    public boolean inRect(double x0, double x1, double y0, double y1) {
        if (field.x < Math.min(x0,x1) || field.x > Math.max(x0,x1)) return false;
        if (field.y < Math.min(y0,y1) || field.y > Math.max(y0,y1)) return false;
        return true;
    }

    public void initMotors(HardwareMap hwMap) {
        initMotors(hwMap, "lf", "lb", "rb", "rf");
    }
    
    public void initMotors(HardwareMap hwMap, String lfName, String lbName, String rbName, String rfName) {
        lf = initDriveMotor(hwMap, lfName, DcMotor.Direction.FORWARD);
        lb = initDriveMotor(hwMap, lbName, DcMotor.Direction.FORWARD);
        rb = initDriveMotor(hwMap, rbName, DcMotor.Direction.REVERSE);
        rf = initDriveMotor(hwMap, rfName, DcMotor.Direction.REVERSE);
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
        if (disableDrive) return;
        double denominator = Math.max(Math.abs(ry) + Math.abs(rx) + Math.abs(rw), 1);
        double lfPower = (rx - ry - rw) / denominator;
        double rfPower = (rx + ry + rw) / denominator;
        double lbPower = (rx + ry - rw) / denominator;
        double rbPower = (rx - ry + rw) / denominator;

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
    
    public void driveXYW(double rx, double ry, double rw) {
        driveXYW(rx, ry, rw, 1);
    }
    
    public boolean driveXYH(double rx, double ry, double fh) {
        driveXYW(rx, ry, calcRVW(fh), 1);
        return false;
    }
    
    public boolean driveFieldXYW(double fx, double fy, double rw, double vel) {
        double heading = Math.toRadians(getHeading());
        
        double rx = fx * Math.cos(-heading) - fy * Math.sin(-heading);
        double ry = fx * Math.sin(-heading) + fy * Math.cos(-heading);
        driveXYW(rx, ry, rw, vel);
        return false;
    }

    public void configurePinpoint(HardwareMap hwMap) {
        pinpoint = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setOffsets(70.946, -103.718);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // if we try to set field pose here, we'll reset on every opmode init
    }

    public void updatePinpoint() {
        pinpoint.update();
        field = Pose.toPose(pinpoint.getPosition());
        field.x += fieldXOffset;
        field.y += fieldYOffset;
    }
    
    public void setPinpointPosition(double fpx, double fpy, double fph) {
        fieldXOffset = 0;
        fieldYOffset = 0;
        field = new Pose(fpx, fpy, fph);
        pinpoint.setPosition(field.toPose2D());
    }
    
    public double getPinpointHeading() {
        return Math.toDegrees(pinpoint.getHeading());
    }
    
    public double moveToXYH(double tx, double ty, double th, double vel) {
        return moveToXYW(tx, ty, calcRVW(th), vel);
    }
    public double moveToXYH(double tx, double ty, double th, double vel, double hGain) {
        return moveToXYW(tx, ty, calcRVW(th, hGain), vel);
    }
    
    public double moveToXYW(double tx, double ty, double rw, double vel) {
        double fdx = tx - field.x;
        double fdy = ty - field.y;
        double dist = Math.hypot(fdx, fdy);
        double absHeadingRad = Math.atan2(fdy, fdx);
        double relHeadingRad = absHeadingRad - Math.toRadians(getHeading());
        double rdx = Math.cos(relHeadingRad) * dist;
        double rdy = Math.sin(relHeadingRad) * dist;
        double dScale = Math.abs(rdx) + Math.abs(rdy);
        double rx = rdx / dScale;
        double ry = rdy / dScale;

        driveXYW(rx * vel, ry * vel, rw);
        return dist;
    }

    public boolean moveTargetXYH(double tx, double ty, double th, Ramp ramp,
                                double dist) {
        double td = distTo(tx, ty);
        return moveTargetXYH(tx, ty, th, ramp.scale(td), dist, 360, 0);
    }
    public boolean moveTargetXYH(double tx, double ty, double th, Ramp ramp,
                                double dist, double ang) {
        double td = distTo(tx, ty);
        return moveTargetXYH(tx, ty, th, ramp.scale(td), dist, ang, 0);
    }
    public boolean moveTargetXYH(double tx, double ty, double th, Ramp ramp,
                                double dist, double ang, double hGain) {
        double td = distTo(tx, ty);
        return moveTargetXYH(tx, ty, th, ramp.scale(td), dist, ang, 0, hGain);
    }
    public boolean moveTargetXYH(double tx, double ty, double th, double vel,
                                double dist) {
        return moveTargetXYH(tx, ty, th, vel, dist, 360, 0);
    }    
    public boolean moveTargetXYH(double tx, double ty, double th, double vel,
                                double dist, double ang) {
        return moveTargetXYH(tx, ty, th, vel, dist, ang, 0);
    }
    public boolean moveTargetXYH(double tx, double ty, double th, double vel,
                                double dist, double ang, double sec) {
        return moveTargetXYH(tx, ty, th, vel, dist, ang, 0, 0.006);
    }
    public boolean moveTargetXYH(double tx, double ty, double th, double vel,
                                double dist, double ang, double sec, double hGain) {
        double d = moveToXYH(tx, ty, th, vel, hGain);
        if (d > dist || !isHeading(th, ang)) {
            atTargetTimer.reset();
            return false;
        }
        if (atTargetTimer.seconds() < sec) return false;
        stop();
        return true;
    }
    
    public Supplier<Boolean> moveStep(double fx, double fy, double fh, double vel, double dist) {
        return () -> moveToXYH(fx, fy, fh, vel) < dist && stop();
    }

    public boolean isHeading(double fh, double ang) {
        return Math.abs(AngleUnit.normalizeDegrees(fh - getHeading())) < ang;
    }
    
    public double calcRVW(double th) {
        return calcRVW(th, 0.006);
    }
    public double calcRVW(double th, double gain) {
        return Range.clip(AngleUnit.normalizeDegrees(th - getHeading()) * gain, -0.3, 0.3);
    }

    public boolean stop() {
        driveXYW(0, 0, 0);
        return true;
    }

    public void configureOtos(HardwareMap hwMap) {
        myOtos = hwMap.get(SparkFunOTOS.class, "otos");
        
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-44.0/25.4, 148.0/25.4, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(0.995);
        myOtos.setAngularScalar(3600.0/3615.0);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        // If we reset field pose here, it will reset every opmode init
        // SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        // field = Pose.toPose(currentPosition);
        // myOtos.setPosition(currentPosition);
    }
    
    public void updateOTOS() {
        field = Pose.toPose(myOtos.getPosition());
    }
    
    public void setOtosPosition(double fpx, double fpy, double fph) {
        fieldXOffset = 0;
        fieldYOffset = 0;
        field = new Pose(fpx, fpy, fph);
        myOtos.setPosition(new SparkFunOTOS.Pose2D(fpx, fpy, fph));
    }
    
    // public void setXYOffset(double x, double y) {
    //     // SparkfunOTOS.Pose2D pos = myOtos.getPosition();
    //     Pose pos = Pose.toPose(odo.getPosition());
    //     fieldXOffset = x - pos.x;
    //     fieldYOffset = y - pos.y;
    // }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("lf: ", TelemetryEx.format(lf));
        telemetry.addData("rf: ", TelemetryEx.format(rf));
        telemetry.addData("lb: ", TelemetryEx.format(lb));
        telemetry.addData("rb: ", TelemetryEx.format(rb));
        telemetry.addData("pos: ", String.format("(%.2f, %.2f) %.2f", field.x, field.y, field.h));
        // Pose2D pose = odo.getPosition();
        // telemetry.addData("odoPos: ", String.format("(%.2f, %.2f) %.2f", 
        //     pose.getX(DistanceUnit.INCH), 
        //     pose.getY(DistanceUnit.INCH),
        //     pose.getHeading(AngleUnit.DEGREES)));
    }
    
    public static class Pose {
        public double x;
        public double y;
        public double h;
        
        public Pose() {}
        
        public Pose(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }
        
        public static Pose toPose(Pose2D pose) {
            Pose ret = new Pose();
            ret.x = pose.getX(DistanceUnit.INCH);
            ret.y = pose.getY(DistanceUnit.INCH);
            ret.h = pose.getHeading(AngleUnit.DEGREES);
            return ret;
        }
        public static Pose toPose(SparkFunOTOS.Pose2D pose) {
            Pose ret = new Pose();
            ret.x = pose.x;
            ret.y = pose.y;
            ret.h = pose.h;
            return ret;
        }
        
        public Pose2D toPose2D() {
            return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h);
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
}
