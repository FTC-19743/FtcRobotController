package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="Autonomous Warehouse Blue")
public class AutoWarehouseBlue extends LinearOpMode {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    Robot robot;

    @Override
    public void runOpMode() {
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.BLUE;
        teamUtil.telemetry.addLine("Initializing Op Mode...please wait");


        teamUtil.telemetry.update();


        robot = new Robot();
        robot.armsCalibrated =false;
        //TSE Detector and all assemblies are intialized
        robot.init(true);
        robot.detector.initialize();
        robot.detector.activate();
        int lastDetection = 0;
        int newDetection;
        while (!opModeIsActive()) {
            teamUtil.pause(250);
            newDetection = robot.detector.warehouseDetect();

            if (newDetection > 0) {
                lastDetection = newDetection;
            }

            telemetry.addData("Detection Value: ", lastDetection);
            telemetry.update();


        }



        waitForStart();
        //takes a starting IMU reading and a starting SystemTime reading
        double startingIMU = robot.drive.getIMUHeading();
        long startingTime = System.currentTimeMillis();

        if (lastDetection == 1) {
            robot.outakeArm.runToFirstLevelAuto();

        }
        else if (lastDetection == 2) {
            robot.outakeArm.runToSecondLevelAuto();
        }
        else if (lastDetection == 3) {
            robot.outakeArm.runToThirdLevelAuto();
        }
        robot.drive.moveInches(.35,3.5);
        robot.drive.spinRightWithIMUV2(35,.4);
        robot.drive.moveInches(.3,25);

        robot.outakeArm.spinnerOutput();
        teamUtil.pause(1000);
        robot.outakeArm.spinnerStop();
        robot.drive.moveBackInches(.25,9);
        robot.drive.spinLeftWithIMUV2(115,.3);
        robot.outakeArm.runToSharedHub();
        robot.drive.moveInches(.45,40);


        //FROM HERE BELOW IS EXPERIMENTAL; PICKING UP FREIGHT CODE
        //uses starting IMU value to figure out where to turn (so that it is oriented towards warehouse)
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
        robot.drive.moveInches(.45,44);
        robot.drive.spinLeftWithIMUV2(65,.25);
        robot.drive.moveInches(.4,7);
        robot.outakeArm.spinnerOutput();
        teamUtil.pause(1000);
        robot.outakeArm.spinnerStop();
        robot.drive.moveBackInches(.4,6);
        robot.drive.spinLeftWithIMUV2(110,.45);
        robot.outakeArm.runToSharedHub();
        robot.drive.moveInches(.75,52);
        //uses system time to identify whether or not crash is likely
        long timeLeft = 30000-(System.currentTimeMillis()-startingTime);
        robot.outakeArm.runToGround();
        teamUtil.pause(timeLeft-1250);





    }



}