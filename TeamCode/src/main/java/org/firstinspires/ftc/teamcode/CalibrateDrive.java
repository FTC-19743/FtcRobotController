package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.teamcode.assemblies.Robot23;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@TeleOp(name="CalibrateDrive", group="Linear Opmode")

public class CalibrateDrive extends LinearOpMode {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    Robot23 robot;
    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;


    public void runOpMode() {

        teamUtil.init(this);


        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        armsGamepad = new TeamGamepad();
        armsGamepad.initilize(false);


        telemetry.addLine("Ready to start");
        telemetry.update();

        robot = new Robot23();
        robot.initialize();
        robot.calibrate();
        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();
        robot.drive.setTargetPositionToleranceAllMotors(10);
        robot.drive.setNewPIDCoefficients();
        PIDFCoefficients frontLeftPID = robot.drive.frontLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        log("Front Left PID" + frontLeftPID);

        while(opModeIsActive()){

            driverGamepad.loop();
            armsGamepad.loop();

            if(driverGamepad.wasYPressed()){
                robot.drive.moveCM(0.6,15);

            }
            if(driverGamepad.wasAPressed()){
                robot.drive.backCM(0.6,15);

            }
            if(driverGamepad.wasXPressed()){
                robot.drive.strafeLeft(0.6,140);

            }

            if(driverGamepad.wasBPressed()){

                robot.drive.strafeRight(0.6,140);
            }

            if(driverGamepad.wasLeftPressed()){
                robot.drive.spinLeftToHeading(225,0.6);
            }

            if(driverGamepad.wasRightPressed()){
                robot.drive.spinRightToHeading(135,0.6);
            }

            if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
                robot.drive.setHeading(180);
            }

            robot.drive.outputTelemetry();
            telemetry.update();
        }


    }
}
