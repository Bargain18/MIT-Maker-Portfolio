
// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.hardware.dfrobot.HuskyLens;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.util.ElapsedTime;

// @TeleOp

// public class ATestOp extends LinearOpMode {

//     NavCS nav;
//     Nav.Path outPath;
//     Nav.Path inPath;
//     Nav.Pose inPose0;
//     DualPad gpad;

//     @Override
//     public void runOpMode() {
        
//         nav = new NavCS();
//         telemetry.addData("Status", "hardware init");
//         telemetry.update();
//         nav.init(hardwareMap);
//         nav.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         telemetry.addData("Status", "camera init");
//         telemetry.update();
//         nav.initCamera(hardwareMap);
//         // while (!isStopRequested()) {
//         //     telemetry.addData("Status", "set manual exposure");
//         //     telemetry.update();
//         //     // if (nav.setManualExposure(this, 0, 30, 250)) break;
//         //     sleep(20);
//         // }
        
//         outPath = new Nav.Path();
//         outPath.addPose(new Nav.Pose(-11, 36,-1))
//                 .addPose(new Nav.Pose(-11, 12,0.25))
//                 .addPose(new Nav.Pose(-11,-36,-1))
//                 .addPose(new Nav.Pose(-28,-54,-1));
               
//         inPose0 = new Nav.Pose(-24,40,-1,2,0.6);
//         inPath = new Nav.Path();
//         inPath.addPose(inPose0)
//                 .addPose(new Nav.Pose(-11,-36,-1, 2, 0.6))
//                 .addPose(new Nav.Pose(-11,18,-1))
//                 .addPose(new Nav.Pose(-24,40,0.2));
        
//         gpad = new DualPad();
//         gpad.init();

//         telemetry.addData("Status", "Initialized");
//         telemetry.update();
//         // Wait for the game to start (driver presses PLAY)
//         while (!opModeIsActive() && !isStopRequested()) {
//             gpad.update(gamepad1, gamepad2);
//             nav.updateAprilTagPosition(0, telemetry);
//             nav.updateAprilTagPosition(1, telemetry);
//             if (gpad.x) nav.setHeading(0.25);
//             if (gpad.dpad_down) { nav.setAlliance(1); }
//             if (gpad.a) { nav.setAlliance(2); }
//             telemetry.addData("propPos", nav.getPropPos());
//             telemetry.addData("satMax", nav.propProc.satMax);
//             nav.addTelemetry(telemetry);
//             telemetry.update();
//         }
        
//         nav.propEnable(false);

//         nav.setHeading(0.25);
//         nav.setYX(-24, 48);
//         HuskyLens.Block foundBlock = null;
    
//         while (opModeIsActive()) {
//             gpad.update(gamepad1, gamepad2);
//             nav.updateTracking();
//             nav.updateAprilTagPosition(0, telemetry);
//             foundBlock = nav.getHusky();
//             if (gpad.x) { out4(); }
//             if (gpad.b) { in4(); }
//             if (gpad.start && !gpad.previous.start) {
//                 nav.setHeading(0.25);
//                 nav.setYX(-24,48);
//             }
            
//             double ry = -gpad.left_stick_y - gpad.right_stick_y;
//             double rx = gpad.right_stick_x;
//             double rw = -gpad.left_stick_x;
            
//             nav.driveYXW(ry,rx,rw,0.6);
  
//             if (foundBlock != null) {
//                 telemetry.addData("husky", foundBlock);      
//                 telemetry.addData("husky2", "%d,%d,%d,%d", foundBlock.left, foundBlock.width, foundBlock.top, foundBlock.height);
//             }
//             nav.addTelemetry(telemetry);
//             telemetry.addData("updates", nav.updates);
//             telemetry.update();
//         }
//     }
    
//     public void out4() {
//         nav.setPath(outPath);
//         while (opModeIsActive()) {
//             gpad.update(gamepad1, gamepad2);
//             if (!gpad.x) break;
//             nav.updateTracking();
//             nav.updateAprilTagPosition();
//             nav.followPath();
//             nav.addTelemetry(telemetry);
//             telemetry.update();
//         }
//         nav.driveStop();
//     }
    
//     public void in4() {
//         inPose0.x = nav.field.x;
//         inPose0.y = nav.field.y;
//         nav.setPath(inPath);
//         while (opModeIsActive()) {
//             gpad.update(gamepad1, gamepad2);
//             if (!gpad.b) break;
//             nav.updateTracking();
//             nav.updateAprilTagPosition();
//             nav.addTelemetry(telemetry);
//             telemetry.update();
//         }
//         nav.driveStop();
//     }
// }
