package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.RobotValues.*;
import java.util.List;

@TeleOp

public class Bteleop extends LinearOpMode {

    int bpress = 0;
    RobotHardware bot;
    Dispatcher dx;
    DualPad gpad;
    boolean gyroLock = true;
    boolean prevDriving = false;
    int count = 1;
    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }        
        bot = new RobotHardware();
        bot.init(hardwareMap);
        // ITDLocalize.initCamera(hardwareMap);
        dx = bot.dx;
        dx.set("/specimen/intakeSide", SPECGRAB_FRONT);
        dx.set("/robot/localizeSwitch", true);
        
        dx.subscribe(HAS_SAMPLE, i -> {gamepad1.rumble(1, 1, 200); gamepad2.rumble(1, 1, 200);});
        
        gpad = new DualPad();
        gpad.init();
        gyroLock = true;
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            gpad.update(gamepad1, gamepad2);
            bot.updateTracking();
            if (gpad.shift.start && !gpad.previous.shift.start) bot.led5Mode++;
            bot.dx.run();
            bot.addTelemetry(telemetry);
            telemetry.update();
        }
        bot.pushbar.setPosition(PUSHBAR_READY);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            gpad.update(gamepad1, gamepad2);
            bot.updateTracking();
            if (gpad.guide && !gpad.previous.guide && !gpad.back) {
                dx.set(ROBOTTASK, bot.task.SETUP);
            }
            if (gpad.shift.start && !gpad.previous.shift.start) bot.led5Mode++;
            if (gpad.x) {
                dx.set(ROBOTTASK, bot.task.SPEC_SCORE);
            }
            if (gpad.a && !gpad.left_bumper) {
                dx.set(SPECIMENPHASE, bot.phase.FORCE_RETRACT);
            }
            if (gpad.shift.a && !gpad.previous.shift.a && !gpad.left_bumper) {
                double i = SPECGRAB_FRONT;
                if (gpad.shift.left_trigger > 0.5) { i = SPECGRAB_BACK; }
                dx.set("/specimen/intakeSide", i);
                dx.set(SPECIMENPHASE, bot.phase.INTAKE);
            }
            if (gpad.gamepad1.b && !gpad.previous.gamepad1.b) {
                bpress++;
                dx.set(ROBOTTASK, bot.task.GRABNGO);
            }
            if (gpad.shift.b && !gpad.previous.shift.b) {
                bot.nextClawWrist();
            }

            if (gpad.gamepad1.y && !gpad.left_bumper) {
                if (gpad.left_trigger > 0.5) dx.set(SAMPLEPHASE, bot.phase.EXTEND_FRONT);
                else dx.set(SAMPLEPHASE, bot.phase.EXTEND_OBS);
            }
            if (gpad.gamepad1.shift.y && !gpad.left_bumper) {
                dx.set(SAMPLEPHASE, bot.phase.EXTEND45);
            }
            if (gpad.gamepad2.shift.y && !gpad.left_bumper) {
                dx.set(SPECIMENPHASE, bot.phase.MANUAL);
                bot.specarm.setPosition(SPECARM_SAVE);
            }
            // if (gpad.gamepad1.back && !gpad.previous.gamepad1.back && !gpad.guide) {
            //     dx.set(ROBOTTASK, bot.task.SPEC_CYCLE);
            // }
            if (gpad.right_trigger > 0.4) {
                dx.set(ROBOTTASK, bot.task.IDLE);
                dx.set(SAMPLEPHASE, bot.phase.SEARCH);
                if (bot.inRect(-10, 12.5, -25, -33) && dx.isEqual(SPECIMENPHASE, bot.phase.READY)) 
                    dx.set(SPECIMENPHASE, bot.phase.RETRACT);
            }
            // if (dx.isEqual(SPECIMENPHASE, bot.phase.READY)
            //         && bot.inRect(-74, 74, -36, -48)
            //         && !bot.hasSpecimen()) {
            //     // auto-retract specimen if it is empty
            //     dx.set(SPECIMENPHASE, bot.phase.RETRACT);            
            // }
            if (gpad.shift.right_trigger > 0.4) {
                dx.set(SAMPLEPHASE, bot.phase.OUTTAKE);
            }
            if (gpad.gamepad1.right_bumper && !gpad.gamepad1.left_bumper) {
                if (!bot.inRect(35, 74, -62, -74))
                    bot.cancelTasks();
                dx.set(SAMPLEPHASE, bot.phase.RELEASE);
            }
            if (gpad.shift.dpad_left) {
                bot.setAlliance(3);
            }
            if (gpad.shift.dpad_right) {
                bot.setAlliance(1);
            }
            
            if(gpad.gamepad2.right_stick_x > 0.01 || gpad.gamepad2.right_stick_y > 0.01) {
                double position = Math.atan2(gpad.gamepad2.right_stick_y, gpad.gamepad2.right_stick_x) / Math.PI;
                bot.clawwrist.setPosition((position + 1) / 2);
            }

            if (gpad.back && gpad.guide) {
                if (bot.getFacing() == bot.NORTH)
                    bot.setPosition(63, -65, 0);
                else if (bot.getFacing() == bot.WEST)
                    bot.setPosition(71, -58, 90);
            }

            double slideVelocity = 0;
            if (gpad.dpad_down) slideVelocity = -2500*.95;
            if (gpad.dpad_up) slideVelocity = 2500*.95;
            bot.adjustSlide(slideVelocity);
            
            double tiltVelocity = 0;
            if (gpad.shift.dpad_up) tiltVelocity = 2000;
            if (gpad.shift.dpad_down) tiltVelocity = -1000;
            bot.adjustTilt(tiltVelocity);

            double fx = gpad.gamepad1.right_stick_x;
            double fy = -gpad.gamepad1.right_stick_y;
            double rw = -gpad.gamepad1.left_stick_x;
            boolean driving = (Math.abs(fx) > 0.009 || Math.abs(fy) > 0.009 || Math.abs(rw) > 0.009);
            if (driving || prevDriving) {
                double speed = (gpad.gamepad1.left_trigger > 0.4) ? 0.1 : 0.7;
                if(dx.isEqual(SAMPLEPHASE, bot.phase.SEARCH) || dx.isEqual(SAMPLEPHASE, bot.phase.INTAKE))
                {
                    if(bot.inRect(-36,36,-36,36)) 
                        speed = 0.2; 
                }
                if (dx.isEqual(SAMPLEPHASE, bot.phase.EXTEND45) && bot.inRect(-40, -75, -40, -75))
                    speed = speed*0.4;
                if (!dx.isEqual(ROBOTTASK, bot.task.GRABNGO)) bot.cancelTasks();
                bot.driveFieldXYW(fx, fy, rw, speed);
            }
            prevDriving = driving;
            bot.dx.run();
            
            telemetry.addData("bpress", bpress);
            telemetry.addData("postcount", dx.postcount);
            telemetry.addData("repostcount", dx.repostcount);
            telemetry.addData("joystick", "%f %f %f", fx, fy, rw);
            telemetry.addData("driving", driving);
            telemetry.addData("prevdriving", prevDriving);
            bot.addTelemetry(telemetry);
            bot.dx.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
