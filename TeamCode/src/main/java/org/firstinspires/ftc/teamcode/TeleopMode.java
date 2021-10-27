package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name="TeleopMode", group="Linear Opmode")
public class TeleopMode extends LinearOpMode {

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

        while (opModeIsActive()) {
            robot.drive.manualControl(-gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
