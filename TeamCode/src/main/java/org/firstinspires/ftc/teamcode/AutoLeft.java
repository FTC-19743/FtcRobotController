package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.OpenCVSignalDetector;
import org.firstinspires.ftc.teamcode.assemblies.Robot23;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
@Autonomous(name="AutoLeft")
public class AutoLeft extends LinearOpMode {
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
        robot.drive.initializePIDFCoefficients();
        robot.outake.closeGrabber();
        int defaultThreshold = robot.drive.backLeft.getTargetPositionTolerance();
        signalDetector = new OpenCVSignalDetector();
        signalDetector.initialize(true, true);
        signalDetector.activate();
        teamUtil.LEFT = true;




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


        robot.newAutoV5(true, signalDetector.signalDetect());
        //robot.halfwayJointTest();














    }
}
