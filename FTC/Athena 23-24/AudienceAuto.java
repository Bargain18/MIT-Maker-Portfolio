package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class AudienceAuto extends BackdropAuto {
    Nav.Path inAudienceBlue = new Nav.Path()
                .addPose(11, -12, 2001)
                .addPose(11, 20, 0.287)
                .addPose(15, 33, 0.287)
                .addPose(24, 35.5, 0.287);
    Nav.Path inAudienceRed = new Nav.Path() 
            .addPose(-11, -12, 2001)
            .addPose(-11, 13, 0.2)
            .addPose(-15, 24, 0.16);
    
    @Override
    public void run1() {
        scorePurple(17, 46, -35.5, 0.085, 0.35);
        bot.setLevel(26);
        bot.resetMode(bot.mode.EXTEND);
        Nav.Path yellow = new Nav.Path()
            .addPose(30, -37, 0, 1.5, 0.3)
            .addPose(30, -37, 2001, 3, 0.3)
            .addPose(11, -37, 2001, nav.defaultPoseLR, 0.4);
        yellow.addAll(inAudienceBlue);
        cycleIn(yellow, 25);
        scoreYellow(41.9, 42, 0.5);
        cycleFull(blueCycleOut, 3, blueCycleIn, 15);
        cycleFull(blueCycleOut, 1, blueCycleIn, 15);
        park();
    }
    
    @Override
    public void run2() {        
        scorePurple(18, 48.5, -39, 0, 0.35); // should be level 18...?
        bot.setLevel(26);
        bot.resetMode(bot.mode.EXTEND);
        Nav.Path yellow = new Nav.Path()
            .addPose(32, -48, 2001, 1.5, 0.3)
            .addPose(32, -48, 2001, 6, 0.3)
            .addPose(11, -48, 2001);
        yellow.addAll(inAudienceBlue);
        cycleIn(yellow, 25);
        scoreYellow(37, 42, 0.5);
        cycleFull(blueCycleOut, 3, blueCycleIn, 15);
        cycleFull(blueCycleOut, 1, blueCycleIn, 15);
        park();
    }
    
    @Override
    public void run3() {
        scorePurple(22, 50, -38.5, -0.065, 0.35);
        bot.setLevel(26);
        bot.resetMode(bot.mode.EXTEND);
        Nav.Path yellow = new Nav.Path()
            .addPose(47, -35, 0.0, 1.5, 0.3)
            .addPose(47, -35, 0.0, 1.5, 0.3)
            .addPose(13, -35, 2001);
        yellow.addAll(inAudienceBlue);
        cycleIn(yellow, 25);
        scoreYellow(31, 42, 0.5);
        cycleFull(blueCycleOut, 3, blueCycleIn, 15);
        cycleFull(blueCycleOut, 1, blueCycleIn, 27);
        park();
    }
    
    @Override
    public void run4() {
        scorePurple(20, -47, -41, -0.47, 0.35);
        bot.setLevel(26);
        bot.resetMode(bot.mode.EXTEND);
        Nav.Path yellow = new Nav.Path()
            .addPose(-47, -35, 0.5, 1.5, 0.3)
            .addPose(-47, -35, 0.5, 1.5, 0.3)
            .addPose(-18, -35, 2001);
        yellow.addAll(inAudienceRed);
        cycleIn(yellow, 25);
        scoreYellow(-27.5, 42, 0.5);
        cycleFull(redCycleOut, 3, redCycleIn, 15);
        cycleFull(redCycleOut, 1, redCycleIn, 27);
        park();
    }
    
    @Override
    public void run5() {
        scorePurple(21, -46.5, -39, -0.56, 0.35);
        bot.setLevel(26);
        bot.resetMode(bot.mode.EXTEND);
        Nav.Path yellow = new Nav.Path()
            .addPose(-32, -48, 2001, 1.5, 0.3)
            .addPose(-32, -48, 2001, 6, 0.3)
            .addPose(-11, -48, 2001);
        yellow.addAll(inAudienceRed);
        cycleIn(yellow, 25);
        scoreYellow(-33.5, 42, 0.5);
        cycleFull(redCycleOut, 3, redCycleIn, 15);
        cycleFull(redCycleOut, 1, redCycleIn, 15);
        park();
    }
    
    @Override
    public void run6() {
        scorePurple(22, -47, -36.5, -0.61, 0.35);
        bot.setLevel(26);
        bot.resetMode(bot.mode.EXTEND);
        Nav.Path yellow = new Nav.Path()
            .addPose(-30, -36.5, 0.5, 1.5, 0.3)
            .addPose(-30, -36.5, 2001, nav.defaultPoseLR, 0.3)
            .addPose(-15, -36.5, 2001);
        yellow.addAll(inAudienceRed);
        cycleIn(yellow, 25);
        //waitFor(3000);
        scoreYellow(-39.5, 42, 0.7);
        cycleFull(redCycleOut, 3, redCycleIn, 15);
        cycleFull(redCycleOut, 1, redCycleIn, 15);
        park();
    }
    
    @Override
    public int getRandomization() {
        // use camera selected path if none requested by driver
        return nav.getPropPos() + (alliance == 2 ? 4 : -2);
    }
    
    @Override
    public void setAuton(int a, int p) {
        alliance = a;
        propPos = p;
        bot.setLEDPattern(alliance);
        nav.setAlliance(alliance, (alliance == 2 ? 0 : 3));
        if (alliance == 2) {
            nav.setHeading(-0.5);
            nav.setYX(-60, -39);
        }
        if (alliance == 1) {
            nav.setHeading(0);
            nav.setYX(60, -39);
       }
    }
}
