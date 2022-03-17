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

        //TSE Detector and all assemblies are intialized
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
            //Decides which level to print based on detection
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
        //takes a starting IMU reading and a starting SystemTime reading
        double startingIMU = robot.drive.getIMUHeading();
        long startingTime = System.currentTimeMillis();
        //logs time for reference
        String startingTimeToLog = String.format("%.2f", robot.drive.getIMUHeading());

        log(startingTimeToLog);



        //raises arm to corresponding level based on detection
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
        //drops off pre-loaded freight
        robot.outakeArm.spinnerOutput();
        teamUtil.pause(1200);
        robot.outakeArm.spinnerStop();
        robot.drive.moveBackInches(.25,9);
        robot.drive.spinRightWithIMUV2(115,.3);
        //runs arm to shared level to be able to make it over barriers
        robot.outakeArm.runToSharedHub();
        robot.drive.moveInches(.45,43);

        //FROM HERE BELOW IS EXPERIMENTAL; PICKING UP FREIGHT CODE
        double degreesNeeded = startingIMU+45;
        double degreesNeededInverted = degreesNeeded*-1;
        //takes in freight from the warehouse
        robot.outakeArm.runToGround();
        robot.drive.spinRightWithIMUV2(37.5, .25);
        robot.outakeArm.spinnerIntake();
        robot.drive.moveInches(.25,16);
        robot.outakeArm.runToThirdLevel();
        robot.drive.moveBackInches(.25,15);
        robot.outakeArm.spinnerStop();
        /* //experimental IMU code that needs to be further reviewed
        double currentImu=robot.drive.getIMUHeading();
        double degreesNeededForTurn = startingIMU+100;

         */
        //returns to alliance shipping hub
        robot.drive.spinRightWithIMUV2(135,.25);
        String currentIMU = String.format("%.2f", robot.drive.getIMUHeading());

        log(currentIMU);
        robot.drive.moveInches(.45,46);
        robot.drive.spinRightWithIMUV2(63,.25);
        robot.drive.moveInches(.4,8);
        //drops off newly acquired freight
        robot.outakeArm.spinnerOutput();
        teamUtil.pause(1000);
        robot.outakeArm.spinnerStop();
        robot.drive.moveBackInches(.4,6);
        robot.drive.spinRightWithIMUV2(120,.45);
        robot.drive.moveInches(.55,51);
        //uses starting system time value to identify when to stop robot in case a crash is possible
        long timeLeft = 30000-(System.currentTimeMillis()-startingTime);
        robot.outakeArm.runToGround();
        teamUtil.pause(timeLeft-1250);


    }



}