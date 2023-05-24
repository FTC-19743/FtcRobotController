package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libs.PixyCam2;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "testPixyCam")
public class testPixyCam extends LinearOpMode {
    TeamGamepad gamepad;
    private Servo servo;


    public void runOpMode() {
        PixyCam2 pixyCam = hardwareMap.get(PixyCam2.class, "pixycam");
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);

        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            if (gamepad.wasUpPressed()) {
                pixyCam.getVersionInfo();
            }
            telemetry.addLine("PixyCam: HW:"+ pixyCam.getHWVersion()+ " FW:" + pixyCam.getFWVersion()+" Build:" + pixyCam.getBuildNum()+ " Type:" + pixyCam.getFWType());
            telemetry.update();
        }
    }
}
