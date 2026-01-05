package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Extension {
    public DcMotorEx slide1;
    public DcMotorEx slide2;
    public DcMotorEx slide3;
    
    ElapsedTime timer = null;
    int targetPos;
    double integralSum;
    double lastError;
    double pow;
    double vel;
    double lasttpower = 0;
    
    double integralLimit = 0;
    
    public double kP = 10;
    double kI = 0;
    double kD = 0;
    
    public Extension(HardwareMap hardwareMap) {
        slide1 = hardwareMap.get(DcMotorEx.class, "slide");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        slide3 = hardwareMap.get(DcMotorEx.class, "slide3");
        
        slide1.setDirection(DcMotor.Direction.REVERSE);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        slide2.setDirection(DcMotor.Direction.REVERSE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        slide3.setDirection(DcMotor.Direction.REVERSE);
        slide3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        slide1.setZeroPowerBehavior(zeroPowerBehavior);
        slide2.setZeroPowerBehavior(zeroPowerBehavior);
    }
    
    public void setMode(DcMotor.RunMode mode) {
        timer = null;
        slide1.setMode(mode);
        slide2.setMode(mode);
    }
    
    public int getCurrentPosition() {
        return slide1.getCurrentPosition();
    }
    
    public boolean isBusy() {
        return slide1.isBusy();
    } 
    
    public void setPower(double power) {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setPower(power);
        slide2.setPower(power);
    }
    
    public void torquePower(double power) {
        if (lasttpower == 0 && power == 0) return;
        setPower(power);
        slide3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide3.setPower(power);
        lasttpower = power;
    }
    
    public void setVelocity(double vel) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setVelocity(vel);
        slide2.setVelocity(vel);
        this.vel = vel;
    }
    
    public void setTargetPosition(int targetPos) {
        setTargetPosition(targetPos, 1);
    }
    
    public void setTargetPosition(int targetPos, double pow) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        timer = new ElapsedTime();
        this.targetPos = targetPos;
        this.pow = pow;
        integralSum = 0;
        lastError = 0;
    }
    
    public void update() {
        if (timer == null) return;
        double dt = timer.seconds();
        timer.reset();
        double error = targetPos - getCurrentPosition();
        double derivative = (error - lastError) / dt;
        integralSum = integralSum + (error * dt);
        // integralSum = Range.clip(integralSum, -integralLimit, integralLimit);
        
        double out = ((kP * error) + (kI * integralSum) + (kD * derivative)) * pow;
        slide1.setVelocity(out);
        slide2.setVelocity(out);
        this.vel = out;
    }
    
    public String toString() {
        long currentPos = getCurrentPosition();
        String mode = "P->" + targetPos;
        if (timer == null) {
            switch (slide1.getMode()) {
                case RUN_TO_POSITION:
                    mode = "P->"+targetPos; break;
                case RUN_USING_ENCODER:
                    mode = "U"; break;
                case RUN_WITHOUT_ENCODER:
                    mode = "W"; break;
                case STOP_AND_RESET_ENCODER:
                    mode = "Z"; break;
            }
        }
        double amps = 0;
        String busy = isBusy() ? "busy" : "";
        return String.format("%d %s %4.1fA %6.1f %s",
                          currentPos, mode, amps, vel, busy);
    }
}