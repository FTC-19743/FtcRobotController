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

@Autonomous(name="Autonomous Warehouse Blue")
public class AutoWarehouseBlue extends LinearOpMode {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
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

            if (newDetection > 0) {
                lastDetection = newDetection;
            }

            telemetry.addData("Detection Value: ", lastDetection);
            telemetry.update();


        }



        waitForStart();
        double startingIMU = robot.drive.getIMUHeading();

        if (lastDetection == 1) {
            robot.outakeArm.runToFirstLevelAuto();

        }
        else if (lastDetection == 2) {
            robot.outakeArm.runToSecondLevelAuto();
        }
        else if (lastDetection == 3) {
            robot.outakeArm.runToThirdLevelAuto();
        }
        robot.drive.moveInches(.25,4);
        robot.drive.spinRightWithIMUV2(30,.35);
        robot.drive.moveInches(.25,22);

        robot.outakeArm.spinnerOutput();
        teamUtil.pause(1000);
        robot.outakeArm.spinnerStop();
        robot.drive.moveBackInches(.25,6);
        robot.drive.spinLeftWithIMUV2(115,.3);
        robot.outakeArm.runToSharedHub();
        robot.drive.moveInches(.45,40);


        //FROM HERE BELOW IS EXPERIMENTAL; PICKING UP FREIGHT CODE
        double degreesNeeded = startingIMU-45;
        double degreesNeededInverted = degreesNeeded*-1;
        robot.outakeArm.runToGround();
        robot.drive.spinLeftWithIMUV2(degreesNeededInverted, .25);
        robot.outakeArm.spinnerIntake();
        robot.drive.moveInches(.25,15);
        robot.outakeArm.spinnerStop();
        robot.outakeArm.runToThirdLevel();
        robot.drive.moveBackInches(.25,15);
        robot.drive.spinLeftWithIMUV2(135,.25);
        robot.drive.moveInches(.45,40);
        robot.drive.spinLeftWithIMUV2(65,.25);
        robot.drive.moveInches(.4,7);
        robot.outakeArm.spinnerOutput();
        teamUtil.pause(1000);
        robot.outakeArm.spinnerStop();
        robot.drive.moveBackInches(.35,6);
        robot.drive.spinLeftWithIMUV2(120,.4);
        robot.drive.moveInches(.45,40);
        robot.outakeArm.runToGround();
        teamUtil.pause(2000);





    }



}