package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.lynx.LynxModule;

@TeleOp

public class SlideTest extends LinearOpMode {
    
    RobotHardware bot;
    DualPad gpad;
    
    int targetPosition = 1000;
    
    public void runOpMode() {
        bot = new RobotHardware();
        bot.init(hardwareMap);
        gpad = new DualPad();
        gpad.init();
        
        waitForStart();
        
        while (opModeIsActive()) {
            gpad.update(gamepad1, gamepad2);
            bot.dx.run();
            
            double power = 0;
            
            if (gpad.dpad_up && !gpad.previous.dpad_up) targetPosition += 100;
            if (gpad.dpad_down && !gpad.previous.dpad_down) targetPosition -= 100;
            if (gpad.dpad_right && !gpad.previous.dpad_right) 
                bot.slide.kP += 0.1;
            else if (gpad.dpad_left && !gpad.previous.dpad_left) 
                bot.slide.kP -= 0.1;
                
            if (power != 0) bot.slide.setPower(power);
            
            if (gpad.a && !gpad.previous.a) bot.slide.setTargetPosition(targetPosition);
            
            telemetry.addData("slide", bot.slide.getCurrentPosition());
            telemetry.addData("target", targetPosition);
            telemetry.addData("kP", bot.slide.kP);
            telemetry.update();
        }
    }
}