package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Collections;
import java.util.Comparator;
import java.util.ArrayList;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.RobotValues.*;
import java.util.List;
import org.opencv.core.Point;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class LimeLightTest extends LinearOpMode {
    Limelight limelight;
    RobotHardware bot;
    Dispatcher dx;
    DualPad gpad;
    int pipeline = 7;
    
    @Override
    public void runOpMode() {
        bot = new RobotHardware();
        bot.init(hardwareMap);
        bot.initLimelight(hardwareMap);
        sleep(500);
        dx = bot.dx;
        gpad = new DualPad();
        gpad.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        
        // dx.set(SAMPLEPHASE, bot.phase.SEARCH);

        while (opModeIsActive()) {
            gpad.update(gamepad1, gamepad2);
            bot.updateTracking();
            bot.dx.run();
            bot.limelight.update();
            
            if (gpad.right_trigger > 0.4) 
                dx.set(SAMPLEPHASE, bot.phase.SEARCH);
            if (gpad.b)
                dx.set(SAMPLEPHASE, bot.phase.INTAKECLAW);
            
            if (gpad.a && !gpad.previous.a) {
                bot.limelight.processTargetPos();
            }
            
            if (gpad.guide && !gpad.previous.guide) { bot.pinpoint.resetPosAndIMU(); }
            
            if (gpad.a) {
                if (bot.limelight.targetPos != null) {
                    Limelight.LLDetection targetPos = bot.limelight.targetPos;

            //         bot.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //         bot.goPosition(bot.slide, targetPos.slide, 0.6);
            //         bot.slideTilt(targetPos.slide);
            //         bot.clawwrist.setPosition(targetPos.angle);

                    double th = bot.getHeading();
                    double tx = bot.field.x + targetPos.xDist * Math.cos(Math.toRadians(th));
                    double ty = bot.field.y + targetPos.xDist * Math.sin(Math.toRadians(th));

            //         bot.moveTargetXYH(tx, ty, th, 0.2, 1);
                    
                    telemetry.addData("targetPos", String.format("(%.2f, %.2f) %.2f", tx, ty, th));
                    telemetry.addData("targetSlide", targetPos.slide);
                }
            } else {
            //     bot.stop();
            //     bot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //     bot.slide.setPower(0);
            //     bot.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            
            // double tiltVelocity = 0;
            // if (gpad.dpad_up) tiltVelocity = 2000;
            // if (gpad.dpad_down) tiltVelocity = -1000;
            // bot.adjustTilt(tiltVelocity);
            
            if (gpad.dpad_right && !gpad.previous.dpad_right) bot.limelight.slideOffset += 10;
            if (gpad.dpad_left && !gpad.previous.dpad_left) bot.limelight.slideOffset -= 10;
            
            bot.limelight.addTelemetry(telemetry);
            telemetry.addData("pos", String.format("(%.2f,%.2f) %.2f", bot.field.x, bot.field.y, bot.field.h));
            telemetry.addData("slide", bot.slide.getCurrentPosition());
            telemetry.addData("slideOffset", bot.limelight.slideOffset);
            telemetry.addData("pipeline", pipeline);
            telemetry.update();
        }
    }
}