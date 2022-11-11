package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.FourWheelDrive;
import org.firstinspires.ftc.teamcode.assemblies.Outake;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.Robot23;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name="MecanumTeleopTest", group="Linear Opmode")

public class MecanumTeleopTest extends LinearOpMode {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    Robot23 robot;
    TeamGamepad gamepad;

    public void runOpMode() {
        teamUtil.init(this);


        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);






        telemetry.addLine("Ready to start");
        telemetry.update();

        robot = new Robot23();
        robot.initialize();
        robot.calibrate();
        telemetry.addLine("Ready to start");
        telemetry.update();
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        boolean dPadUpWasPressed = false;
        boolean dPadDownWasPressed = false;
        boolean dPadLeftWasPressed = false;
        boolean dPadRightWasPressed = false;
        double powerFactor = 1;

        robot.drive.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.drive.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.drive.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.drive.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();


            double y = -teamUtil.theOpMode.gamepad1.left_stick_y; // Remember, this is reversed!
            double x = teamUtil.theOpMode.gamepad1.left_stick_x*1.1 ; // Counteract imperfect strafing
            double rx = teamUtil.theOpMode.gamepad1.right_stick_x;

            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            /*
            //working code for robot centric drive
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx);
            double backLeftPower = (y + x + rx) ;
            double frontRightPower = (y - x - rx) ;
            double backRightPower = (y + x - rx) ;

             */

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            if (gamepad1.right_trigger > .8) {
                powerFactor = 1;
            }else{
                powerFactor = .4;
            }
                robot.drive.frontLeft.setPower(frontLeftPower*powerFactor);
                robot.drive.backLeft.setPower(backLeftPower*powerFactor);
                robot.drive.frontRight.setPower(frontRightPower*powerFactor);
                robot.drive.backRight.setPower(backRightPower*powerFactor);



            if (gamepad1.start==true) {
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                // Without this, data retrieving from the IMU throws an exception
                imu.initialize(parameters);


            }

            if (gamepad2.dpad_up){
                robot.outake.runPulleyUp();
            }
            else if(gamepad2.dpad_down){
                robot.outake.runPulleyDown();
            }
            else{
                robot.outake.stop();
            }
            if(gamepad2.left_trigger>0.8){
                robot.outake.openGrabber();
            }
            if(gamepad2.right_trigger>0.8){
                robot.outake.closeGrabber();
            }

            robot.outputTelemetry();
            telemetry.update();
            }
        }


}


