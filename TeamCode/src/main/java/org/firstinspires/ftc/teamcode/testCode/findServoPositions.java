package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name = "findServoPositions")

public class findServoPositions extends LinearOpMode {
    public static final double MAJOR_INCREMENT = 0.05;
    public static final double MINOR_INCREMENT = 0.01;
    public static double INITIAL_POS = .5;
    public double currentPosition = INITIAL_POS;
    TeamGamepad gamepad;
    private Servo servo;


    public void runOpMode() {
        servo = hardwareMap.servo.get("servo");
        servo.setPosition(INITIAL_POS);
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            if (gamepad.wasUpPressed() && (currentPosition < 1)) {
                currentPosition = currentPosition + MAJOR_INCREMENT;
                servo.setPosition(currentPosition);
            }
            if (gamepad.wasDownPressed() && (currentPosition > 0)) {
                currentPosition = currentPosition - MAJOR_INCREMENT;
                servo.setPosition(currentPosition);

            }
            if (gamepad.wasLeftPressed() && (currentPosition < 1)) {
                currentPosition = currentPosition + MINOR_INCREMENT;
                servo.setPosition(currentPosition);
            }
            if (gamepad.wasRightPressed() && (currentPosition > 0)) {
                currentPosition = currentPosition - MINOR_INCREMENT;
                servo.setPosition(currentPosition);

            }
            telemetry.addData("position: ", servo.getPosition());
            telemetry.update();
        }
    }
}



