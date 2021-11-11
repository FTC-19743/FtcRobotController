package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

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

        waitForStart();
        //code for blue alliance
            if(teamUtil.alliance == teamUtil.Alliance.BLUE){
                log("move inches ready");
                //goes forward to drop off the freight that you start with
                robot.drive.moveInches(.5,19);

                robot.drive.spinRightWithIMU(90,.5);
                /*
                //drops off the freight that you start with
                robot.drive.moveInches(.5,9);
                //goes to the carousel
                robot.drive.moveBackInches(.5, 2);
                robot.drive.spinRightWithIMU(80,.5);
                robot.drive.moveInches(.5,14);
                //spins the duck off of the carousel
                robot.spinner.spinOnce();
                //moves back into the storage area for the parking bonus
                robot.drive.moveBackInches(.5,16);
                robot.drive.spinLeftWithIMU(70, .5);
                */
            }
            //code for red alliance
            else{
                //goes forward to drop off the freight that you start with
                robot.drive.moveInches(.5,16);

                robot.drive.spinLeftWithIMU(90,.5);
                robot.spinner.spinOnce();
                /*

                //drops off the freight that you start with
                robot.drive.moveInches(.5,9);
                //goes to the carousel
                robot.drive.moveBackInches(.5, 2);
                robot.drive.spinLeftWithIMU(80,.5);
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
