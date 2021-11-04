package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.assemblies.TwoWheelDrive;

@Autonomous(name="Autonomous V1")
public class AutomousopMode extends LinearOpMode {


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


        robot.drive.spinRightWithIMU(90, .1);
    }
}
