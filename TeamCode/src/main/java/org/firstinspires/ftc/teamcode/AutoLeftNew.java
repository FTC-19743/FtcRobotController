package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.OpenCVSignalDetector;
import org.firstinspires.ftc.teamcode.assemblies.Robot23;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Autonomous(name="AutoLeftNew")
public class AutoLeftNew extends LinearOpMode {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    Robot23 robot;
    public OpenCVSignalDetector signalDetector;

    @Override
    public void runOpMode() {
        teamUtil.init(this);
        robot = new Robot23();
        robot.initialize();
        robot.calibrate();
        robot.outake.closeGrabber();

        signalDetector = new OpenCVSignalDetector();
        signalDetector.initialize(true);
        signalDetector.activate();




        telemetry.addLine("Waiting for start");
        telemetry.update();

        long now = System.currentTimeMillis();
        while (!opModeIsActive()) {
            signalDetector.writeTelemetry();
            sleep(100); // save cpu for OpenCV
            if (System.currentTimeMillis() > now + 2000) {
                signalDetector.nextView();
                now = System.currentTimeMillis();
            }
        }
        signalDetector.deactivate();



        waitForStart();
        robot.drive.setHeading(180);

        robot.outake.runToLevelNoWait(3);

        robot.drive.strafeRight(.6,142);


        robot.drive.spinLeftToHeading(220,.6);

        robot.drive.backCM(.6,12);

        robot.outake.openGrabber();
        teamUtil.pause(500);
        robot.drive.moveCM(.6,17);
        robot.outake.runToBottomNoWait(true);
        robot.drive.spinRightToHeading(180,.6);
        robot.drive.moveCM(.6,50);
        robot.drive.strafeLeftToLine();
        robot.drive.setAllMotorsToSpeed(0.3);
        teamUtil.pause(1000);
        robot.drive.setAllMotorsToSpeed(0);

        robot.outake.closeGrabber();
        teamUtil.pause(500);
        robot.outake.runToLevelNoWait(3);
        robot.drive.backCM(.3,70);
        robot.drive.spinLeftToHeading(225,.6);
        robot.drive.backCM(.6,13);

        robot.outake.openGrabber();
        teamUtil.pause(500);

        robot.drive.moveCM(.6,15);
        robot.outake.runToBottomNoWait(true);

        robot.drive.spinRightToHeading(180,.6);


        int detection = signalDetector.signalDetect();
        if(detection == 1){
            robot.drive.moveCM(.6,60);
        }else if(detection == 2){

        }else{
            robot.drive.backCM(.6,60);
        }

        if(robot.drive.getHeading()>90){
            robot.drive.spinRightToHeading(180,0.3);
        }
        else{
            robot.drive.spinLeftToHeading(180,0.3);
        }













    }




}
