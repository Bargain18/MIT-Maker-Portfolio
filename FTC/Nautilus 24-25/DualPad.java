package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;


public class DualPad {

    static DualPad emptyPad = new DualPad();
    
    DualPad previous;
    DualPad shift;
    DualPad gamepad1;
    DualPad gamepad2;
    
    public float left_stick_x;
    public float left_stick_y;
    public float right_stick_x;
    public float right_stick_y;
    public boolean dpad_up;
    public boolean dpad_down;
    public boolean dpad_left;
    public boolean dpad_right;
    public boolean a;
    public boolean b;
    public boolean x;
    public boolean y;
    public boolean guide;
    public boolean start;
    public boolean back;
    public boolean left_bumper;
    public boolean right_bumper;
    public boolean left_stick_button;
    public boolean right_stick_button;
    public float left_trigger;
    public float right_trigger;
    public int id;
    public long timestamp;
    
    public void init() {
        previous = new DualPad();
        shift = new DualPad();
        gamepad1 = new DualPad();
        gamepad2 = new DualPad();
        previous.shift = new DualPad();
        previous.gamepad1 = new DualPad();
        previous.gamepad2 = new DualPad();
        gamepad1.shift = new DualPad();
        gamepad2.shift = new DualPad();
        previous.gamepad1.shift = new DualPad();
        previous.gamepad2.shift = new DualPad();
        
        // map uninitialized references to objects above
        // this allows ".shift.gamepad1" to be an alias for ".gamepad1.shift"
        // "gamepad1.previous" to be an alias for "previous.gamepad1", etc.
        // and avoids null pointer dereferences

        /* previous.previous = previous;

        shift.previous = previous.shift;
        shift.shift = shift;
        shift.gamepad1 = gamepad1.shift;
        shift.gamepad2 = gamepad2.shift;
        
        gamepad1.previous = previous.gamepad1;
        gamepad1.gamepad1 = gamepad1;
        gamepad1.gamepad2 = gamepad2;
        
        gamepad2.previous = previous.gamepad2;
        gamepad2.gamepad1 = gamepad1;
        gamepad2.gamepad2 = gamepad2;
        
        previous.shift.previous = previous.shift;
        previous.shift.shift = previous.shift;
        previous.shift.gamepad1 = previous.gamepad1.shift;
        previous.shift.gamepad2 = previous.gamepad2.shift;
        
        previous.gamepad1.previous = previous.gamepad1;
        previous.gamepad1.gamepad1 = previous.gamepad1;
        previous.gamepad1.gamepad2 = previous.gamepad2;
        
        previous.gamepad2.previous = previous.gamepad2;
        previous.gamepad2.gamepad1 = previous.gamepad1;
        previous.gamepad2.gamepad2 = previous.gamepad2;
        
        gamepad1.shift.previous = previous.gamepad1.shift;
        gamepad1.shift.shift = gamepad1.shift;
        gamepad1.shift.gamepad1 = gamepad1.shift;
        gamepad1.shift.gamepad2 = gamepad2.shift;
        
        gamepad2.shift.previous = previous.gamepad2.shift;
        gamepad2.shift.shift = gamepad2.shift;
        gamepad2.shift.gamepad1 = gamepad1.shift;
        gamepad2.shift.gamepad2 = gamepad2.shift;
        
        previous.gamepad1.shift.previous = previous.gamepad1.shift;
        previous.gamepad1.shift.shift = previous.gamepad1.shift;
        previous.gamepad1.shift.gamepad1 = previous.gamepad1.shift;
        previous.gamepad1.shift.gamepad2 = previous.gamepad2.shift;
        
        previous.gamepad2.shift.previous = previous.gamepad2.shift;
        previous.gamepad2.shift.shift = previous.gamepad2.shift;
        previous.gamepad2.shift.gamepad1 = previous.gamepad1.shift;
        previous.gamepad2.shift.gamepad2 = previous.gamepad2.shift;
        */
    }
    
    public void copyshift(Gamepad gp) {
        if (gp.left_bumper) {
            this.fastcopy(emptyPad);
            shift.fastcopy(gp);
            left_stick_x = gp.left_stick_x;
            left_stick_y = gp.left_stick_y;
            right_stick_x = gp.right_stick_x;
            right_stick_y = gp.right_stick_y;
            this.timestamp = (long)gp.timestamp;
        } else {
            this.fastcopy(gp);
            shift.fastcopy(emptyPad);
            this.timestamp = (long)gp.timestamp;
        }
    }
    
    public void merge(DualPad gp1, DualPad gp2) {
        dpad_up = gp1.dpad_up || gp2.dpad_up;
        dpad_down = gp1.dpad_down || gp2.dpad_down;
        dpad_left = gp1.dpad_left || gp2.dpad_left;
        dpad_right = gp1.dpad_right || gp2.dpad_right;
        a = gp1.a || gp2.a;
        b = gp1.b || gp2.b;
        x = gp1.x || gp2.x;
        y = gp1.y || gp2.y;
        guide = gp1.guide || gp2.guide;
        start = gp1.start || gp2.start;
        back = gp1.back || gp2.back;
        left_bumper = gp1.left_bumper || gp2.left_bumper;
        right_bumper = gp1.right_bumper || gp2.right_bumper;
        left_stick_button = gp1.left_stick_button || gp2.left_stick_button;
        right_stick_button = gp1.right_stick_button || gp2.right_stick_button;
        left_trigger = gp1.left_trigger;
        right_trigger = gp1.right_trigger;
        left_stick_x = gp1.left_stick_x;
        left_stick_y = gp1.left_stick_y;
        right_stick_x = gp1.right_stick_x;
        right_stick_y = gp1.right_stick_y;
        if (left_trigger == 0) left_trigger = gp2.left_trigger;
        if (right_trigger == 0) right_trigger = gp2.right_trigger;
        if (left_stick_x == 0) left_stick_x = gp2.left_stick_x;
        if (left_stick_y == 0) left_stick_y = gp2.left_stick_y;
        if (right_stick_x == 0) right_stick_x = gp2.right_stick_x;
        if (right_stick_y == 0) right_stick_y = gp2.right_stick_y;
        timestamp = Math.max(gp1.timestamp, gp2.timestamp);
    }
    
    public void update(Gamepad gp1, Gamepad gp2) {
        previous.fastcopy(this);
        previous.shift.fastcopy(shift);
        previous.gamepad1.fastcopy(gamepad1);
        previous.gamepad1.shift.fastcopy(gamepad1.shift);
        previous.gamepad2.fastcopy(gamepad2);
        previous.gamepad2.shift.fastcopy(gamepad2.shift);

        gamepad1.copyshift(gp1);
        gamepad2.copyshift(gp2);
        this.merge(gamepad1, gamepad2);
        shift.merge(gamepad1.shift, gamepad2.shift);
    }
    
    public void fastcopy(Gamepad gp) {
        left_stick_x = gp.left_stick_x;
        left_stick_y = gp.left_stick_y;
        right_stick_x = gp.right_stick_x;
        right_stick_y = gp.right_stick_y;
        dpad_up = gp.dpad_up;
        dpad_down = gp.dpad_down;
        dpad_left = gp.dpad_left;
        dpad_right = gp.dpad_right;
        a = gp.a;
        b = gp.b;
        x = gp.x;
        y = gp.y;
        guide = gp.guide;
        start = gp.start;
        back = gp.back;
        left_bumper = gp.left_bumper;
        right_bumper = gp.right_bumper;
        left_stick_button = gp.left_stick_button;
        right_stick_button = gp.right_stick_button;
        left_trigger = gp.left_trigger;
        right_trigger = gp.right_trigger;
        id = gp.id;
        timestamp = gp.timestamp;
    }

    public void fastcopy(DualPad gp) {
        left_stick_x = gp.left_stick_x;
        left_stick_y = gp.left_stick_y;
        right_stick_x = gp.right_stick_x;
        right_stick_y = gp.right_stick_y;
        dpad_up = gp.dpad_up;
        dpad_down = gp.dpad_down;
        dpad_left = gp.dpad_left;
        dpad_right = gp.dpad_right;
        a = gp.a;
        b = gp.b;
        x = gp.x;
        y = gp.y;
        guide = gp.guide;
        start = gp.start;
        back = gp.back;
        left_bumper = gp.left_bumper;
        right_bumper = gp.right_bumper;
        left_stick_button = gp.left_stick_button;
        right_stick_button = gp.right_stick_button;
        left_trigger = gp.left_trigger;
        right_trigger = gp.right_trigger;
        id = gp.id;
        timestamp = gp.timestamp;
    }
}
