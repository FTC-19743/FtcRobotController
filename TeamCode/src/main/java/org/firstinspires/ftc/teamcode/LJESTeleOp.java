package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.Robot23;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name="LJESTeleOp", group="Linear Opmode")

public class LJESTeleOp extends LinearOpMode {
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




        waitForStart();

        while (opModeIsActive()) {
            driverGamepad.loop();










            //declaration of power and denominator variables for math
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;
            double denominator;

            double y = gamepad1.left_stick_y*.7; // Remember, this is reversed!
            double x = gamepad1.left_stick_x*1.1 ; // Counteract imperfect strafing
            double rx = (-gamepad1.right_stick_x)*0.5;
            if(Math.abs(rx)<0.15){
                rx=0;
            }
            //Orientation anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double botHeading = -Math.toRadians((robot.drive.getHeading()-180)); //
            //double botHeading = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle; //removed negative
            telemetry.addLine("botheading: " + Math.toDegrees(botHeading));

            //telemetry.addLine("current cup level "+ String.valueOf(cupLevel));

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]



            //working code for robot centric drive

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;



            double powerFactor = 0.3;
            robot.drive.frontLeft.setPower(-frontLeftPower*powerFactor);
            robot.drive.backLeft.setPower(-backLeftPower*powerFactor);
            robot.drive.frontRight.setPower(-frontRightPower*powerFactor);
            robot.drive.backRight.setPower(-backRightPower*powerFactor);

            if (gamepad1.dpad_up){
                robot.outake.runPulleyUp();
            }

            else if(gamepad1.dpad_down){
                robot.outake.runPulleyDown();
            }

            if(driverGamepad.wasLeftTriggerPressed()) {
                if (robot.outake.grabber.getPosition() >= robot.outake.GRAB - 0.01 && robot.outake.grabber.getPosition() <= robot.outake.GRAB + 0.01) {
                    robot.outake.openGrabber();
                } else {
                    robot.outake.closeGrabber();
                }
            }



            robot.outputTelemetry();
            telemetry.update();

        }
    }


}


