package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.TwoWheelDrive;

@Autonomous(name="Autonomous V1")
public class AutomousopMode extends LinearOpMode {
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
            telemetry.addData("Detection Value: ", lastDetection);
            telemetry.update();


        }


        waitForStart();
        //code for blue alliance
        if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
            log("move inches ready");
            //goes forward to drop off the freight that you start with
            robot.drive.moveInches(.5, 19);

            robot.drive.spinRightWithIMUV2(90, .5);

            //drops off the freight that you start with
            robot.drive.moveInches(.5, 9);
            robot.drive.moveBackInches(.5, 2);
            //goes to the carousel
            robot.drive.spinLeftWithIMUV2(80, .5);
            robot.drive.moveBackInches(.5, 14);
            //spins the duck off of the carousel
            robot.spinner.spinOnce();
            //moves back into the storage area for the parking bonus
            robot.drive.moveInches(.5, 16);
            robot.drive.spinLeftWithIMUV2(70, .5);

        }
        //code for red alliance
        else {
            if (lastDetection != 0) {
                robot.outakeArm.spinnerOutput();
                teamUtil.pause(100);
                robot.outakeArm.spinnerStop();
                robot.drive.moveInches(.3, 36);
                //teamUtil.pause(500);
                if(lastDetection == 1) {
                    robot.outakeArm.runArmToPosition(robot.outakeArm.Level1);
                } else if(lastDetection == 2){
                    robot.outakeArm.runArmToPosition(robot.outakeArm.Level2);
                } else{
                    robot.outakeArm.runArmToPosition(robot.outakeArm.Level3);
                }

                robot.drive.spinRightWithIMUV2(90, .1);

                if(lastDetection == 1) {
                    robot.drive.moveInches(.25,2);
                } else if(lastDetection == 2){
                    robot.drive.moveInches(.25,5);
                } else{
                    robot.drive.moveInches(.25,10.5);
                }
                robot.outakeArm.spinnerOutput();
                teamUtil.pause(1000);
                robot.outakeArm.spinnerStop();


                robot.outakeArm.runToTop();
                if(lastDetection == 1) {
                    robot.drive.moveBackInches(.4,26.5);
                } else if(lastDetection == 2){
                    robot.drive.moveBackInches(.4,29.5);
                } else{
                    robot.drive.moveBackInches(.4,34.5);
                }

                teamUtil.pause(1000);
                robot.drive.moveInches(.25,5);
                robot.drive.spinRightWithIMUV2(90,.1);
                robot.drive.moveInches(.35,28);

                teamUtil.pause(1000);
                robot.drive.spinRightWithIMUV2(45,.1);
                robot.drive.moveInches(.35,11);
                robot.drive.motorsOn(.05);
                robot.spinner.on(.75);
                teamUtil.pause(3100);
                robot.spinner.off();
                robot.drive.motorsOff();
                robot.drive.spinLeftWithIMUV2(45,.25);
                robot.drive.moveBackInches(.35,17);




                    /*
                    robot.drive.moveInches(.65,1.75);

                    robot.outakeArm.runArmToPosition(robot.outakeArm.Level1);
                    teamUtil.pause(1500);
                    robot.outakeArm.spinnerOutput();

                    teamUtil.pause(2000);
                    robot.outakeArm.spinnerStop();
                    robot.drive.moveBackInches(.3,40);
                    teamUtil.pause(5000);


                }

                //goes forward to drop off the freight that you start with
                robot.drive.moveInches(.5,24);
                robot.outakeArm.runArmToPosition(robot.outakeArm.Level3);
                robot.drive.spinRightWithIMUV2(38, .25);

                robot.drive.moveInches(.5,11);
                robot.outakeArm.spinnerOutput();
                teamUtil.pause(2000);
                robot.outakeArm.spinnerStop();
                robot.outakeArm.runArmToPosition(robot.outakeArm.Level3);
                robot.drive.moveBackInches(.5,9);
                robot.drive.spinRightWithIMUV2(35,.25);
                robot.drive.moveBackInches(.5,15);
                 */

                //drops off the freight that you start with
                /*
                robot.drive.moveInches(.5,9);
                //goes to the carousel
                robot.drive.moveBackInches(.5, 2);
                robot.drive.spinLeftWithIMUV2(80,.5);
                robot.drive.moveInches(.5,14);
                //spins the duck off of the carousel
                robot.spinner.spinOnce();
                //moves back into the storage area for the parking bonus
                robot.drive.moveBackInches(.5,16);
                robot.drive.spinRightWithIMU(70, .5);
                */
            }


        }
    }
}
