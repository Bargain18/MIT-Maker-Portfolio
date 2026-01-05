package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import static org.firstinspires.ftc.teamcode.RobotValues.*;

@TeleOp

public class ServoTest extends LinearOpMode {

    ServoImplEx servo;
    DualPad gpad;
    
    public void runOpMode() {
        servo = hardwareMap.get(ServoImplEx.class, "wrist");
        // 0 for trapdoor is 0.992
        // servo.setDirection(Servo.Direction.REVERSE);
        String[] names = {"horizontal", "straight"};
        double[] positions = {0.5, 0.842};
        int i = 0;
        
        gpad = new DualPad();
        gpad.init();
        
        waitForStart();
        
        while (opModeIsActive()) {
            gpad.update(gamepad1, gamepad2);
            if (gpad.y && !gpad.previous.y) i++;
            if (gpad.a && !gpad.previous.a) i--;
            
            if (gpad.dpad_up && !gpad.previous.dpad_up) positions[i] += 0.003;
            if (gpad.dpad_down && !gpad.previous.dpad_down) positions[i] -= 0.003;
            if (gpad.shift.dpad_up && !gpad.previous.shift.dpad_up) positions[i] += 0.1;
            if (gpad.shift.dpad_down && !gpad.previous.shift.dpad_down) positions[i] -= 0.1;
            i = i < 0 ? i += names.length : i % names.length;
            
            servo.setPosition(positions[i]);
            telemetry.addData("i: ", i);
            for (int j = 0; j < names.length; j++) {
                String s = String.format("%.3f %c", positions[j], (i == j) ? '<' : ' ');
                telemetry.addData(names[j], s);
            }
            telemetry.update();
        }
    }
}
