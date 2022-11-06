package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name="ButtonTest", group="Linear Opmode")
public class ButtonTest extends LinearOpMode {
    TeamGamepad gamepad;

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    public void runOpMode() {

        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            if(gamepad.wasAPressed()){
                log("A was pressed");
            }
        }
    }
}
