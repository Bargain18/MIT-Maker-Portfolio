    package org.firstinspires.ftc.teamcode;
    
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    @Autonomous
    
    public class AAuton_good extends LinearOpMode {
    
        RobotHardware bot;
        Dispatcher dx;
        DualPad gpad;

        @Override
        public void runOpMode() {
            List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }        
            bot = new RobotHardware();
            bot.init(hardwareMap);
            bot.pinpoint.resetPosAndIMU();
            sleep(500);
            bot.setPosition(15.5, -66, 0);
            dx = bot.dx;
            gpad = new DualPad();
            gpad.init();
    
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            double presetH = -95;
            //0.7 0.25
            DriveBase.Ramp presetRamp = new DriveBase.Ramp(12+3, 6+3, 0.7, 0.25);
            DriveBase.Ramp presetRampTurn = new DriveBase.Ramp(12, 6, 0.9, 0.6);
            // 0.8 0.25
            DriveBase.Ramp pickRamp = new DriveBase.Ramp(12, 6, 0.8, 0.25);
            double presetY0 = -16;
            double presetY1 = -52;
            Dispatcher.Sequence autospec = dx.newSequence("/autospec/step")
                // score specimen 1
                .set("/robot/specscorex", 12.0)
                .set("/robot/specpickx", 15.5)
                .set(ROBOTTASK, bot.task.SPEC_SCORE)
                .waitFor(SPECIMENPHASE, bot.phase.READY)
                .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                .waitFor(ROBOTTASK, bot.task.IDLE)
                .waitFor(() -> bot.moveTargetXYH(22, -46, -5, 0.6, 3))
                .set(SPECIMENPHASE, bot.phase.RETRACT)
                
                .run(() -> bot.pushbar.setPosition(PUSHBAR_READY))
                .waitFor(() -> bot.moveTargetXYH(38, presetY0, presetH, presetRamp, 3))
                .run(() -> bot.pushbar.setPosition(PUSHBAR_PUSH))
                .waitFor(() -> bot.moveTargetXYH(38, presetY1, presetH, presetRamp, 3))
                .run(() -> bot.pushbar.setPosition(PUSHBAR_READY))
                .waitFor(() -> bot.moveTargetXYH(47, presetY0, presetH, presetRamp, 3))
                .run(() -> bot.pushbar.setPosition(PUSHBAR_PUSH))
                .waitFor(() -> bot.moveTargetXYH(47, presetY1, presetH, presetRamp, 3))
                .run(() -> bot.pushbar.setPosition(PUSHBAR_READY))
                .waitFor(() -> bot.moveTargetXYH(55, presetY0, presetH, presetRamp, 3))
                .run(() -> bot.pushbar.setPosition(PUSHBAR_PUSH))
                .waitFor(() -> bot.moveTargetXYH(55, -44, presetH, 0.65, 2))
                
                .waitFor(() -> bot.moveTargetXYH(56, presetY1-1, 0, presetRampTurn, 2, 360, 0.013))
                .run(() -> bot.pushbar.setPosition(PUSHBAR_STOW))
                
                // score specimen #2
                .set("/robot/specscorex", 12.0)
                .set("/robot/specpickx", 55.0)
                .set(ROBOTTASK, bot.task.SPEC_GRAB)
                .waitFor(SPECIMENPHASE, bot.phase.READY)
                .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                .waitFor(ROBOTTASK, bot.task.IDLE)
                //0.8
                .waitFor(() -> bot.moveTargetXYH(15, -43, 0, 0.8, 4))
                .set(SPECIMENPHASE, bot.phase.RETRACT)
            
                // score specimen #3
                .set("/robot/specscorex", 11.0)
                .set("/robot/specpickx", 43.0)
                .waitFor(() -> bot.moveTargetXYH((double) dx.get("/robot/specpickx"), -67, 0, pickRamp, 2))
                .set(ROBOTTASK, bot.task.SPEC_GRAB)
                .waitFor(SPECIMENPHASE, bot.phase.READY)
                .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                .waitFor(ROBOTTASK, bot.task.IDLE)
                .waitFor(() -> bot.moveTargetXYH(15, -43, 0, 0.8, 4))
                .set(SPECIMENPHASE, bot.phase.RETRACT)
                
                // score specimen #4
                .set("/robot/specscorex", 11.0)
                .waitFor(() -> bot.moveTargetXYH((double) dx.get("/robot/specpickx"), -67, 0, pickRamp, 2))
                .set(ROBOTTASK, bot.task.SPEC_GRAB)
                .waitFor(SPECIMENPHASE, bot.phase.READY)
                .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                .waitFor(ROBOTTASK, bot.task.IDLE)
                .waitFor(() -> bot.moveTargetXYH(15, -43, 0, 0.8, 4))
                .set(SPECIMENPHASE, bot.phase.RETRACT)

                // score specimen #5
                .set("/robot/specscorex", 11.0)
                .waitFor(() -> bot.moveTargetXYH((double) dx.get("/robot/specpickx"), -67, 0, pickRamp, 2))
                .set(ROBOTTASK, bot.task.SPEC_GRAB)
                .waitFor(SPECIMENPHASE, bot.phase.READY)
                .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                .waitFor(ROBOTTASK, bot.task.IDLE)
                .waitFor(() -> bot.moveTargetXYH(15, -43, 0, 0.8, 4))
                .set(SPECIMENPHASE, bot.phase.RETRACT)
                
                // score fake specimen #6
                .set("/robot/specscorex", 9.0)
                .waitFor(() -> bot.moveTargetXYH((double) dx.get("/robot/specpickx"), -67, 0, pickRamp, 2))
                .set(ROBOTTASK, bot.task.SPEC_GRAB)
                .waitFor(SPECIMENPHASE, bot.phase.READY)
                .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                .waitFor(ROBOTTASK, bot.task.IDLE)
                .waitFor(() -> bot.moveTargetXYH(15, -43, 0, 0.8, 4))
                .set(SPECIMENPHASE, bot.phase.RETRACT)
                
                // park
                .waitFor(() -> bot.moveTargetXYH((double) dx.get("/robot/specpickx"), -67, 0, pickRamp, 2))
                .waitFor(0.1);
                
            while (!isStarted()) {
                gpad.update(gamepad1, gamepad2);
                bot.updateTracking();
                if (gpad.guide && !gpad.previous.guide) { dx.set(ROBOTTASK, bot.task.SETUP); }
                if (gpad.shift.start && !gpad.previous.shift.start) bot.led5Mode++;
                dx.run();
                bot.addTelemetry(telemetry);
                dx.addTelemetry(telemetry);
                telemetry.update();
            }
    
            waitForStart();
            bot.intakeswing.setPosition(SWING_DOWN);
            bot.setPosition(15.5, -66, 0);
            bot.tiltPosition(TILT_BACK);
            bot.pushbar.setPosition(PUSHBAR_READY);
            
            autospec.start();
            
            while (opModeIsActive()) {
                bot.updateTracking();
                dx.run();
                telemetry.addData("pos: ", String.format("(%.2f, %.2f) %.2f", bot.field.x, bot.field.y, bot.field.h));
                // telemetry.addData("step: ", (int) dx.get("/autospec/step"));
                telemetry.update();
            }
        }
    }
