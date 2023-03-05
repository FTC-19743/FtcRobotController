package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.AprilTagDetector;
import org.firstinspires.ftc.teamcode.assemblies.OpenCVSignalDetector;
import org.firstinspires.ftc.teamcode.assemblies.Robot23;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AutoLeft")
public class AutoLeft extends LinearOpMode {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    Robot23 robot;
    //public OpenCVSignalDetector signalDetector;
    public AprilTagDetector tagDetector;

    @Override
    public void runOpMode() {
        teamUtil.init(this);
        robot = new Robot23();
        robot.initialize();
        robot.calibrate();
        robot.drive.initializePIDFCoefficients();
        robot.outake.closeGrabber();
        int defaultThreshold = robot.drive.backLeft.getTargetPositionTolerance();

        /*
        signalDetector = new OpenCVSignalDetector();
        signalDetector.initialize(true, true);
        signalDetector.activate();
         */
        tagDetector = new AprilTagDetector(1, OpenCvCameraRotation.UPRIGHT, telemetry, hardwareMap);
        tagDetector.initDetector(false);
        tagDetector.activate();
        telemetry.addLine("Waiting for Tag Detector");
        telemetry.update();
        while (!tagDetector.ready.get() ) { // Wait for detector to signal ready
            teamUtil.pause(250);
        }
        tagDetector.startProcessing();

        teamUtil.LEFT = true;

        telemetry.addLine("Waiting for start");
        telemetry.update();

        long now = System.currentTimeMillis();
        while (!opModeIsActive()) {
            //signalDetector.writeTelemetry();
            tagDetector.writeTelemetry();
            sleep(100); // save cpu for OpenCV
            if (System.currentTimeMillis() > now + 2000) {
                tagDetector.writeTelemetry();
                now = System.currentTimeMillis();
            }
        }

        waitForStart();
        int path = tagDetector.signalDetect();
        tagDetector.deactivate();

        robot.newAutoV5(teamUtil.LEFT, path);
        //robot.halfwayJointTest();














    }
}
