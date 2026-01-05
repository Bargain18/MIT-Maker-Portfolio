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
    
    public class AAuton extends LinearOpMode {
    
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
            bot.initLimelight(hardwareMap);
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
                .set("/robot/specscorex", 5.0) // CHANGED FROM 12 TO 5
                .set("/robot/specpickx", 15.5)
                .set(ROBOTTASK, bot.task.SPEC_SCORE)
                .waitFor(SPECIMENPHASE, bot.phase.READY)
                .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                .waitFor(ROBOTTASK, bot.task.IDLE)
                // .waitFor(() -> bot.moveTargetXYH(22, -46, -5, 0.6, 3)) 
                .set(SPECIMENPHASE, bot.phase.RETRACT)
                
                .run(() -> bot.stop())
                .set(ROBOTTASK, bot.task.LIMELIGHT_SEARCH)
                .waitFor(ROBOTTASK, bot.task.IDLE);
                
                // .run(() -> bot.pushbar.setPosition(PUSHBAR_READY))
                // .waitFor(() -> bot.moveTargetXYH(38, presetY0, presetH, presetRamp, 3))
                // .run(() -> bot.pushbar.setPosition(PUSHBAR_PUSH))
                // .waitFor(() -> bot.moveTargetXYH(38, presetY1, presetH, presetRamp, 3))
                // .run(() -> bot.pushbar.setPosition(PUSHBAR_READY))
                // .waitFor(() -> bot.moveTargetXYH(47, presetY0, presetH, presetRamp, 3))
                // .run(() -> bot.pushbar.setPosition(PUSHBAR_PUSH))
                // .waitFor(() -> bot.moveTargetXYH(47, presetY1, presetH, presetRamp, 3))
                // .run(() -> bot.pushbar.setPosition(PUSHBAR_READY))
                // .waitFor(() -> bot.moveTargetXYH(55, presetY0, presetH, presetRamp, 3))
                // .run(() -> bot.pushbar.setPosition(PUSHBAR_PUSH))
                // .waitFor(() -> bot.moveTargetXYH(55, -44, presetH, 0.65, 2))
                
                // .waitFor(() -> bot.moveTargetXYH(56, presetY1-1, 0, presetRampTurn, 2, 360, 0.013))
                // .run(() -> bot.pushbar.setPosition(PUSHBAR_STOW))
                
                // // score specimen #2
                // .set("/robot/specscorex", 12.0)
                // .set("/robot/specpickx", 55.0)
                // .set(ROBOTTASK, bot.task.SPEC_GRAB)
                // .waitFor(SPECIMENPHASE, bot.phase.READY)
                // .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                // .waitFor(ROBOTTASK, bot.task.IDLE)
                // //0.8
                // .waitFor(() -> bot.moveTargetXYH(15, -43, 0, 0.8, 4))
                // .set(SPECIMENPHASE, bot.phase.RETRACT)
                
                // // score specimen #3
                // .set("/robot/specscorex", 11.0)
                // .set("/robot/specpickx", 43.0)
                // .waitFor(() -> bot.moveTargetXYH((double) dx.get("/robot/specpickx"), -67, 0, pickRamp, 2))
                // .set(ROBOTTASK, bot.task.SPEC_GRAB)
                // .waitFor(SPECIMENPHASE, bot.phase.READY)
                // .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                // .waitFor(ROBOTTASK, bot.task.IDLE)
                // .waitFor(() -> bot.moveTargetXYH(15, -43, 0, 0.8, 4))
                // .set(SPECIMENPHASE, bot.phase.RETRACT)
                
                // // score specimen #4
                // .set("/robot/specscorex", 11.0)
                // .waitFor(() -> bot.moveTargetXYH((double) dx.get("/robot/specpickx"), -67, 0, pickRamp, 2))
                // .set(ROBOTTASK, bot.task.SPEC_GRAB)
                // .waitFor(SPECIMENPHASE, bot.phase.READY)
                // .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                // .waitFor(ROBOTTASK, bot.task.IDLE)
                // .waitFor(() -> bot.moveTargetXYH(15, -43, 0, 0.8, 4))
                // .set(SPECIMENPHASE, bot.phase.RETRACT)

                // // score specimen #5
                // .set("/robot/specscorex", 11.0)
                // .waitFor(() -> bot.moveTargetXYH((double) dx.get("/robot/specpickx"), -67, 0, pickRamp, 2))
                // .set(ROBOTTASK, bot.task.SPEC_GRAB)
                // .waitFor(SPECIMENPHASE, bot.phase.READY)
                // .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                // .waitFor(ROBOTTASK, bot.task.IDLE)
                // .waitFor(() -> bot.moveTargetXYH(15, -43, 0, 0.8, 4))
                // .set(SPECIMENPHASE, bot.phase.RETRACT)

                // // score sample
                // .set(SAMPLEPHASE, bot.phase.SEARCH)
                // .run(() -> bot.slidePosition(350))
                // //.waitFor(() -> bot.moveTargetXYH(15, -59, -90, RAMP70, 2))
                // .waitFor(() -> bot.moveTargetXYH(15, -60.5, -90, RAMP40, 2))
                // .set(SAMPLEPHASE, bot.phase.INTAKE)
                // .waitFor(() -> { bot.adjustSlide(1600); return dx.isEqual(SAMPLEPHASE, bot.phase.LIFT); })
                // .set(SAMPLEPHASE, bot.phase.LIFT)
                // .waitFor(0.3)
                // .waitFor(() -> bot.moveTargetXYH(0, -55, -90, 0.8, 6))
                // .set(SAMPLEPHASE, bot.phase.EXTEND45)
                // .waitFor(() -> bot.moveTargetXYH(-62, -55, -45, RAMP80, 3))
                // .waitFor(() -> bot.driveFieldXYW(-1, -1, bot.calcRVW(-45), 0.4), 0.7)
                // .run(() -> bot.stop())
                // // .waitFor(0.3)
                // .set(SAMPLEPHASE, bot.phase.RELEASE)
                // .waitFor(SAMPLEPHASE, bot.phase.RETRACT)
                // .set(SAMPLEPHASE, bot.phase.AUTORETRACT)
                // .waitFor(() -> bot.moveTargetXYH(48, -59, 0, RAMP80, 3))
                // .run(() -> bot.slidePower(0))
                // .waitFor(30)
                
                // // score fake specimen #6
                // .set("/robot/specscorex", 9.0)
                // .waitFor(() -> bot.moveTargetXYH((double) dx.get("/robot/specpickx"), -67, 0, pickRamp, 2))
                // .set(ROBOTTASK, bot.task.SPEC_GRAB)
                // .waitFor(SPECIMENPHASE, bot.phase.READY)
                // .set(ROBOTTASK, bot.task.SPEC_AUTOSCORE)
                // .waitFor(ROBOTTASK, bot.task.IDLE)
                // .waitFor(() -> bot.moveTargetXYH(15, -43, 0, 0.8, 4))
                // .set(SPECIMENPHASE, bot.phase.RETRACT)

                // // park
                // .waitFor(() -> bot.moveTargetXYH((double) dx.get("/robot/specpickx"), -67, 0, pickRamp, 2))
                // .waitFor(0.1);
                
            while (!isStarted()) {
                gpad.update(gamepad1, gamepad2);
                bot.updateTracking();
                if (gpad.guide && !gpad.previous.guide) { dx.set(ROBOTTASK, bot.task.SETUP); }
                if (gpad.shift.start && !gpad.previous.shift.start) bot.led5Mode++;
                dx.run();
                bot.limelight.update();
                bot.addTelemetry(telemetry);
                bot.limelight.addTelemetry(telemetry);
                dx.addTelemetry(telemetry);
                telemetry.update();
            }
    
            waitForStart();
            bot.intakeswing.setPosition(SWING_DOWN);
            bot.setPosition(15.5, -66, 0);
            // bot.tiltPosition(TILT_BACK);
            dx.set(SAMPLEPHASE, bot.phase.SEARCH);
            bot.pushbar.setPosition(PUSHBAR_READY);
            
            autospec.start();
            
            while (opModeIsActive()) {
                bot.updateTracking();
                dx.run();
                bot.limelight.update();
                bot.limelight.addTelemetry(telemetry);
                telemetry.addData("pos: ", String.format("(%.2f, %.2f) %.2f", bot.field.x, bot.field.y, bot.field.h));
                // telemetry.addData("step: ", (int) dx.get("/autospec/step"));
                telemetry.update();
            }
        }
    }
