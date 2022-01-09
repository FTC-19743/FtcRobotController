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
        robot.init();
        robot.detector.activate();
        int lastDetection = 0;
        int newDetection;
        while (!opModeIsActive()) {
            teamUtil.pause(250);
            newDetection = robot.detector.detect();
            if (newDetection > 0) {
                lastDetection = newDetection;
            }
            int detectionValueToPrint=0;
            if(lastDetection == 1){
                detectionValueToPrint=3;
            }
            else if(lastDetection == 2){
                detectionValueToPrint=1;
            }
            else if(lastDetection == 3){
                detectionValueToPrint=2;
            }


            telemetry.addData("Detection Value: ", detectionValueToPrint);
            telemetry.update();


        }


        waitForStart();


        if(lastDetection==1){
            robot.drive.moveInches(.35,17);
            robot.drive.spinLeftWithIMUV2(37,.15);
            robot.outakeArm.runArmToPosition(robot.outakeArm.Level3);
            teamUtil.pause(1750);
            robot.drive.moveInches(.25,8.75);
            robot.outakeArm.spinnerOutput();
            teamUtil.pause(1500);
            robot.outakeArm.spinnerStop();


            //robot.outakeArm.runArmToPosition(robot.outakeArm.Top);
            robot.drive.moveBackInches(.25,8);
            robot.drive.spinLeftWithIMUV2(143,.25);
            robot.drive.moveInches(.35,25);
            robot.drive.spinRightWithIMUV2(95,.25);
            robot.drive.moveBackInches(.35,38);





        } else if(lastDetection==2){
            robot.drive.moveInches(.35,17);
            robot.drive.spinLeftWithIMUV2(37,.15);
            robot.drive.moveBackInches(.25,1.5);
            robot.outakeArm.runArmToPosition(robot.outakeArm.Level1);
            teamUtil.pause(1750);
            robot.drive.moveInches(.25,3.5);
            robot.outakeArm.spinnerOutput();
            teamUtil.pause(1500);
            robot.outakeArm.spinnerStop();
            //robot.outakeArm.runArmToPosition(robot.outakeArm.Top);
            robot.drive.moveBackInches(.25,8);
            robot.drive.spinLeftWithIMUV2(143,.25);
            robot.drive.moveInches(.35,20);
            robot.drive.spinRightWithIMUV2(95,.25);
            robot.drive.moveBackInches(.35,38);



        } else{
            robot.drive.moveInches(.35,17);
            robot.drive.spinLeftWithIMUV2(37,.15);
            robot.outakeArm.runArmToPosition(robot.outakeArm.Level2);
            teamUtil.pause(1750);
            robot.drive.moveInches(.25,5.5);
            robot.outakeArm.spinnerOutput();
            teamUtil.pause(1500);
            robot.outakeArm.spinnerStop();
            //robot.outakeArm.runArmToPosition(robot.outakeArm.Top);
            robot.drive.moveBackInches(.25,10);


            robot.drive.spinLeftWithIMUV2(143,.25);
            robot.drive.moveInches(.35,25);
            robot.drive.spinRightWithIMUV2(95,.25);
            robot.drive.moveBackInches(.35,38);

            //robot.drive.spinLeftWithIMUV2(45,.25);

        }


    }



}