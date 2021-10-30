package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

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
            if(gamepad1.right_bumper == true){
                robot.drive.manualControl(-gamepad1.left_stick_y, gamepad1.right_stick_x);
            }
            else{

                robot.drive.manualControl(-gamepad1.left_stick_y/2, gamepad1.right_stick_x/2);
            }
            if(gamepad1.left_trigger > 0){
                robot.intakeOutput.intakeOutputControl(gamepad1.left_trigger);

            }
            else if(gamepad1.left_bumper){
                robot.intakeOutput.intakeOutputControl(-.75f);

            }
            else{
                robot.intakeOutput.outputStop();
            }
            robot.spinner.carouselControl(gamepad1.right_trigger);
        }
    }
}
