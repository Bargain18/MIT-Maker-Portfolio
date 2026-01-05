package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class TelemetryEx {

    public static String format(DcMotorEx m) {
        int currentPos = m.getCurrentPosition();
        int targetPos = m.getTargetPosition();
        double velocity = m.getVelocity();
        double amps = m.getCurrent(CurrentUnit.AMPS);
        String mode = "?";
        String busy = m.isBusy() ? "busy" : "";
        switch (m.getMode()) {
            case RUN_TO_POSITION:
                mode = "P->"+targetPos; break;
            case RUN_USING_ENCODER:
                mode = "U"; break;
            case RUN_WITHOUT_ENCODER:
                mode = "W"; break;
            case STOP_AND_RESET_ENCODER:
                mode = "Z"; break;
        }
        return String.format("%d %s %4.1fA %6.1f %s",
                                currentPos, mode, amps, velocity, busy);
    }
    
    public static String format(DigitalChannel ch) {
        return format(ch, "false", "true");
    }
    public static String format(DigitalChannel ch, String f) {
        return format(ch, f, "");
    }
    public static String format(DigitalChannel ch, String f, String t) {
        return ch.getState() ? t : f;
    }
    
}