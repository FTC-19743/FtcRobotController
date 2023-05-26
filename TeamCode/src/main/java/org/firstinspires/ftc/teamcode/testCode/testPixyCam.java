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
            if (gamepad.wasLeftPressed()) {
                pixyCam.toggleLEDs(true);
            }
            if (gamepad.wasRightPressed()) {
                pixyCam.toggleLEDs(false);
            }
            if (gamepad.wasDownPressed()) {
                pixyCam.getBlocks((byte)0b11111111, (byte)2);
                //pixyCam.showBug();
            }
            telemetry.addLine(
                    "PixyCam: HW:"+ pixyCam.getHWVersion() +
                            " FW:" + pixyCam.getFWVersionMajor()+
                            "." + pixyCam.getFWVersionMinor()+
                            " Build:" + pixyCam.getBuildNum()+
                            " Type:" + pixyCam.getFWType());
            telemetry.update();
        }
    }
}
// 2022-04-27 07:13:22.441  1684-2383  I2C                     com.qualcomm.ftcrobotcontroller      I  Automatically initializing I2C device PixyCam2 USB (embedded); module 2; bus 1; addr7=0x18
