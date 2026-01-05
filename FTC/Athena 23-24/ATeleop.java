package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class ATeleop extends LinearOpMode {
    
    RobotHardware bot = null;
    Nav nav = null;
    DualPad gpad = null;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        gpad = new DualPad(); gpad.init();
        bot = new RobotHardware(); bot.init(hardwareMap);
        nav = new NavCS(); nav.init(hardwareMap);
        nav.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.launch(false);

        waitForStart();

        while (opModeIsActive()) {
            gpad.update(gamepad1, gamepad2);

            if (gpad.right_trigger > 0.5) {
                if (bot.isMode(bot.mode.IDLE)
                    || bot.isMode(bot.mode.WAIT)
                    || bot.isMode(bot.mode.INTAKE)
                    || bot.isMode(bot.mode.OUTTAKE)
                    || bot.isMode(bot.mode.HOLD)
                    || bot.isMode(bot.mode.INTAKEIDLE)) {
                        bot.setMode(bot.mode.INTAKE);
                    }
                if (bot.isMode(bot.mode.EXTENDED) || bot.isMode(bot.mode.MANUAL)) {
                    if (bot.getBallSw() == 3) bot.setMode(bot.mode.SCORE);
                }
            } else if (gpad.shift.right_trigger > 0.5) {
                if (!bot.isMode(bot.mode.XFER)) bot.setMode(bot.mode.OUTTAKE);
            } else if (bot.isMode(bot.mode.INTAKE) || bot.isMode(bot.mode.OUTTAKE)) {
                bot.setMode(bot.mode.INTAKEIDLE);
            }

            if(gpad.shift.back) bot.setMode(bot.mode.LAUNCH);
            
            if(gpad.x) {
                bot.setMode(bot.mode.HOLD);
            }
            if(gpad.shift.x){
                bot.setMode(bot.mode.XFER);
            }
            if(gpad.right_bumper) {
                bot.setScoreSide( gpad.left_trigger > 0.25 ? 1 : 3);
                bot.setMode(bot.mode.SCORE);
            }
            if(gpad.shift.right_bumper) {
                bot.setScoreSide(2);
                bot.setMode(bot.mode.SCORE);
            }

            double elevVelocity = 0;
            double tiltVelocity = 0;
            if(gpad.dpad_right) elevVelocity = 1000;
            if(gpad.dpad_left) elevVelocity = -1000;
            //defaults
            if (gpad.dpad_up && !gpad.previous.dpad_up) bot.adjustTeleopLevel(+1);
            if (gpad.dpad_down && !gpad.previous.dpad_down) bot.adjustTeleopLevel(-1);

            //manual
            if(gpad.shift.dpad_up) tiltVelocity = 2000;
            if(gpad.shift.dpad_down) tiltVelocity = -1000;
            if (gpad.shift.dpad_right){
                tiltVelocity = 750;
                elevVelocity = 525;
            }
            if (gpad.shift.dpad_left){
                tiltVelocity = -750;
                elevVelocity = -275; 
            }
            if (gpad.right_stick_button && !gpad.previous.right_stick_button){
                bot.setMpPos();
            }
            if (tiltVelocity != 0 || elevVelocity != 0 || bot.isMode(bot.mode.MANUAL)) {
                bot.adjustTilt(tiltVelocity);
                bot.adjustElev(elevVelocity);
            }
            
            if (gpad.y && !bot.isBusy()) {
                bot.setMode(bot.mode.EXTEND);
            }
            
            if (gpad.guide) {
                bot.setMode(bot.mode.STOW);
            }
            
            if (gpad.shift.b) {
                bot.setMode(bot.mode.HANG);
            }
            if (gpad.b) {
                if (gpad.left_trigger > 0.5) bot.setMode(bot.mode.CLIMB);
                else bot.hookDown();
            }
            
            double ry = -gpad.gamepad1.left_stick_y - gpad.gamepad1.right_stick_y;
            double rx = gpad.gamepad1.right_stick_x * 1.5;
            double rw = -gpad.gamepad1.left_stick_x * .6;
            double power = 1 - (gpad.gamepad1.left_trigger * 0.5);
            if(bot.getElevPos() > 100){
                if(ry < 0) ry *= 0.4;
            }
            nav.driveYXW(ry, rx, rw, power);
            bot.update();
            if (bot.leftIntakePixel && bot.rightIntakePixel) {
                gamepad1.rumble(1,1,200);
                gamepad2.rumble(1,1,200);
            }
            telemetry.addData("drive", "%.6f %.6f %.6f", ry, rx, rw);
            nav.addTelemetry(telemetry);
            bot.addTelemetry(telemetry);    
            telemetry.update();
        }
    }
}
