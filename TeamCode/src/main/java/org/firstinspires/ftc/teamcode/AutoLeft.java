package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        signalDetector = new OpenCVSignalDetector();
        signalDetector.initialize(true);
        signalDetector.activate();



        telemetry.addLine("Waiting for start");
        telemetry.update();
        long now = System.currentTimeMillis();
        while (!opModeIsActive())
        {
            signalDetector.writeTelemetry();
            sleep(100); // save cpu for OpenCV
            if (System.currentTimeMillis() > now+2000) {
                signalDetector.nextView();
                now = System.currentTimeMillis();
            }
        }
        signalDetector.deactivate();
        waitForStart();




        telemetry.update();
        robot.outake.closeGrabber();
        teamUtil.pause(500);
        robot.outake.runToMedium();
        robot.outake.pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outake.outputTelemetry();
        telemetry.update();
        robot.outake.pulley.setVelocity(1000);
        robot.drive.strafeLeft(.3, 145);
        robot.drive.strafeRight(.3, 38);
        robot.drive.moveCM(.3,14);
        robot.outake.openGrabber();
        teamUtil.pause(500);
        robot.drive.backCM(.3,16);
        robot.drive.strafeLeft(.3,25);
        robot.drive.spinRightWithIMU(180,0.25);
        robot.drive.moveCM(0.25,35);

        robot.drive.strafeRightToLine();

        robot.outake.runToShort();
        robot.outake.grabber.setPosition(robot.outake.FULLY_OPEN);
        robot.drive.moveCM(.3, 18);
        robot.drive.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drive.frontLeft.setPower(.15);
        robot.drive.frontRight.setPower(.15);
        robot.drive.backLeft.setPower(.15);
        robot.drive.backRight.setPower(.15);
        teamUtil.pause(1500);
        robot.drive.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.backCM(.3, 8);
        robot.outake.runToCupStack();
        robot.outake.grabber.setPosition(robot.outake.GRAB);
        teamUtil.pause(500);
        robot.outake.pulley.setTargetPosition(robot.outake.MEDIUM);
        teamUtil.pause(1000);
        robot.drive.backCM(.3, 30);
        robot.outake.runToShort();
        robot.drive.spinLeftWithIMU(95,.3);
        robot.drive.moveCM(.3, 12);
        robot.outake.grabber.setPosition(robot.outake.OPEN);
        teamUtil.pause(500);
        robot.drive.backCM(.3,12);








        /*

        robot.drive.strafeRight(.3,30);

        if(signalDetector.signalDetect()==1){
            robot.drive.backCM(.3,60);
        }
        else if(signalDetector.signalDetect()==2){
        }
        else{
            robot.drive.moveCM(.3,60);
        }

        robot.outake.pulley.setTargetPosition(10);
        robot.outake.pulley.setVelocity(1000);
        teamUtil.pause(3000);


        robot.drive.spinLeftWithIMU(90,0.25);

         */





    }
}
