package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous Warehouse Red")
public class AutoWarehouseRed extends LinearOpMode {
    public static void log(String logString) {
        RobotLog.d("19743LOG1:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    Robot robot;

    @Override
    public void runOpMode() {
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        teamUtil.telemetry.addLine("Initializing Op Mode...please wait");


        teamUtil.telemetry.update();


        robot = new Robot();
        robot.armsCalibrated =false;
        robot.init(true);
        robot.detector.initialize();
        robot.detector.activate();
        int lastDetection = 0;
        int newDetection;
        while (!opModeIsActive()) {
            teamUtil.pause(250);
            newDetection = robot.detector.detect();
            int lastDetectionToPrint=1;
            if (newDetection > 0) {
                lastDetection = newDetection;
            }
            if(lastDetection==2){
                lastDetectionToPrint=1;
            }
            else if(lastDetection==3){
                lastDetectionToPrint=2;
            }
            else{
                lastDetectionToPrint=3;
            }
            telemetry.addData("Detection Value: ", lastDetectionToPrint);
            telemetry.update();


        }


        waitForStart();
        double startingIMU = robot.drive.getIMUHeading();
        long startingTime = System.currentTimeMillis();
        String startingTimeToLog = String.format("%.2f", robot.drive.getIMUHeading());

        log(startingTimeToLog);




        if (lastDetection == 2) {
            robot.outakeArm.runToFirstLevelAuto();

        }
        else if (lastDetection == 3) {
            robot.outakeArm.runToSecondLevelAuto();
        }
        else if (lastDetection == 1) {
            robot.outakeArm.runToThirdLevelAuto();
        }
        robot.drive.moveInches(.4,6);
        robot.drive.spinLeftWithIMUV2(32,.4);
        robot.drive.moveInches(.3,18.5);

        robot.outakeArm.spinnerOutput();
        teamUtil.pause(1200);
        robot.outakeArm.spinnerStop();
        robot.drive.moveBackInches(.25,9);
        robot.drive.spinRightWithIMUV2(115,.3);
        robot.outakeArm.runToSharedHub();
        robot.drive.moveInches(.45,43);

        //FROM HERE BELOW IS EXPERIMENTAL; PICKING UP FREIGHT CODE
        double degreesNeeded = startingIMU+45;
        double degreesNeededInverted = degreesNeeded*-1;
        robot.outakeArm.runToGround();
        robot.drive.spinRightWithIMUV2(37.5, .25);
        robot.outakeArm.spinnerIntake();
        robot.drive.moveInches(.25,16);
        robot.outakeArm.runToThirdLevel();
        robot.drive.moveBackInches(.25,15);
        robot.outakeArm.spinnerStop();
        double currentImu=robot.drive.getIMUHeading();
        double degreesNeededForTurn = startingIMU+100;
        robot.drive.spinRightWithIMUV2(135,.25);
        String currentIMU = String.format("%.2f", robot.drive.getIMUHeading());

        log(currentIMU);
        robot.drive.moveInches(.45,46);
        robot.drive.spinRightWithIMUV2(63,.25);
        robot.drive.moveInches(.4,8);
        robot.outakeArm.spinnerOutput();
        teamUtil.pause(1000);
        robot.outakeArm.spinnerStop();
        robot.drive.moveBackInches(.4,6);
        robot.drive.spinRightWithIMUV2(120,.45);
        robot.drive.moveInches(.55,51);
        long timeLeft = 30000-(System.currentTimeMillis()-startingTime);
        robot.outakeArm.runToGround();
        teamUtil.pause(timeLeft-1250);


    }



}