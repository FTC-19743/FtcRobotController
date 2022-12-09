package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@TeleOp(name="OutakeSliderTest", group="Linear Opmode")
public class OutakeSlideTest extends LinearOpMode {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    Robot robot;



    @Override
    public void runOpMode() {
        robot = new Robot();
        robot.init(true);
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.dpad_up == true){
                while(gamepad1.dpad_up){
                }
                double positionNeeded = robot.outakeSlide.outakeSlider.getPosition() + .05;
                robot.outakeSlide.outakeSlider.setPosition(positionNeeded);
                telemetry.addData("Outake Slider","Slider Position:%f", robot.outakeSlide.outakeSlider.getPosition());
            }
            else if(gamepad1.dpad_down == true){
                while(gamepad1.dpad_down){
                }
                double positionNeeded = robot.outakeSlide.outakeSlider.getPosition() - .05;
                robot.outakeSlide.outakeSlider.setPosition(positionNeeded);
                telemetry.addData("Outake Slider","Slider Position:%f",robot.outakeSlide.outakeSlider.getPosition());
            }
        }
    }
}


