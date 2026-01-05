package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class DriveBaseTest extends LinearOpMode {
    DriveBaseCS drive;
    DualPad gpad;
    
    DriveBase.Path path = new DriveBase.Path()
                    .addPose(16.5, -50.5, -135, 0.3)
                    .addPose(44, -36, 180, 0.3)
                    .addPose(36, -12, 180, 0.3)
                    .addPose(-54, -12, 180, 0.3)
                    .addPose(-54, -38, 180, 0.3)
                    .addPose(-36, -58, 180, 0.3)
                    .addPose(12, -58, 180, 0.3);
    
    
    @Override
    public void runOpMode() {
        
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        drive = new DriveBaseCS();
        drive.init(hardwareMap);
        drive.initCamera(hardwareMap);
        drive.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPosition(0, 0, 0);
        //drive.setPosition(0, 0, 0);
        
        gpad = new DualPad();
        gpad.init();
        
        waitForStart();
        
        while (opModeIsActive()) {
            gpad.update(gamepad1, gamepad2);
            
            double rx = -gpad.gamepad1.left_stick_y - gpad.gamepad1.right_stick_y;
            double ry = -gpad.gamepad1.right_stick_x * 1.5;
            double rw = -gpad.gamepad1.left_stick_x * .6;
            
            if (gpad.dpad_down) { 
                move(-72, 0, 0, 0.4);
            } else if (gpad.dpad_right) {
                move(-72, 0, 0, 0.6);
            } else if (gpad.dpad_up) {
                move(-72, 0, 0, 0.8);
            } else {
                drive.driveXYW(rx, ry, rw, 0.5);
            }
            
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            
            telemetry.addData("pos", "x=%.2f y=%.2f h=%.2f", drive.field.x, drive.field.y, drive.field.h); 
            telemetry.update();
        }
    }
    
    public void move(double tx, double ty, double th, double vel) {
        double maxVel = vel;
        while (opModeIsActive()) {
            drive.updateOTOS();
            drive.updateAprilTagPosition(telemetry);
            
            double dist = drive.moveToXYH(tx, ty, th, vel);
            if (dist < 1) break;
            vel = Range.scale(dist, 36, 18, 0.8, 0.2);
            vel = Range.clip(vel, 0.2, maxVel);
            
            telemetry.addData("pos", "x=%.2f y=%.2f h=%.2f", drive.field.x, drive.field.y, drive.field.h);
            telemetry.update();
        }
        drive.stop();
    }
 }