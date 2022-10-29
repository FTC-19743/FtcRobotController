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
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name="MecanumTeleopTest", group="Linear Opmode")

public class MecanumTeleopTest extends LinearOpMode {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    Robot23 robot;


    public void runOpMode() {
        teamUtil.init(this);




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

        robot.drive.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.drive.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.drive.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.drive.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            /*
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x; //counteract imperfect strafing
            double rx = gamepad1.right_stick_x; // Remember, this is reversed!

            // Read inverse IMU heading, as the IMU heading is CW positive
            //double botHeading = -imu.getAngularOrientation().firstAngle;
            double botHeading = 1;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

             */

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x*1.1 ; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

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


            robot.drive.frontLeft.setPower(frontLeftPower);
            robot.drive.backLeft.setPower(backLeftPower);
            robot.drive.frontRight.setPower(frontRightPower);
            robot.drive.backRight.setPower(backRightPower);



            if (gamepad1.a == true) {
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                // Without this, data retrieving from the IMU throws an exception
                imu.initialize(parameters);
            }

            if (gamepad1.dpad_up){
                robot.outake.runPulleyUp();
            }
            else if(gamepad1.dpad_down){
                robot.outake.runPulleyDown();
            }
            else{
                robot.outake.stop();
            }

            /*
            if (gamepad1.dpad_up == true) {
                if (!dPadUpWasPressed) {
                    dPadUpWasPressed = true;

                }

            } else {
                if (dPadUpWasPressed) {
                    dPadUpWasPressed = false;
                    teamUtil.log("D-Pad UP Was Bumped");
                    if (gamepad1.right_bumper == true) {
                        robot.drive.MAX_ACCELERATION += 10;
                    } else if (gamepad1.left_bumper == true) {
                        robot.drive.MAX_ACCELERATION -= 10;
                    }

                }
            }

            if (gamepad1.dpad_down == true) {
                if (!dPadDownWasPressed) {
                    dPadDownWasPressed = true;

                }

            } else {
                if (dPadDownWasPressed) {
                    dPadDownWasPressed = false;
                    teamUtil.log("D-Pad DOWN Was Bumped");
                    if (gamepad1.right_bumper == true) {
                        robot.drive.MAX_DECELERATION += 10;
                    } else if (gamepad1.left_bumper == true) {
                        robot.drive.MAX_DECELERATION -= 10;
                    }
                }
            }
            if (gamepad1.dpad_left == true) {
                if (!dPadLeftWasPressed) {
                    dPadLeftWasPressed = true;

                }

            } else {
                if (dPadLeftWasPressed) {
                    dPadLeftWasPressed = false;
                    teamUtil.log("D-Pad LEFT Was Bumped");
                    if (gamepad1.right_bumper == true) {
                        robot.drive.MIN_START_VELOCITY += 10;
                    } else if (gamepad1.left_bumper == true) {
                        robot.drive.MIN_START_VELOCITY -= 10;
                    }
                }

                if (gamepad1.dpad_right == true) {
                    if (!dPadRightWasPressed) {
                        dPadRightWasPressed = true;

                    }

                } else {
                    if (dPadRightWasPressed) {
                        dPadRightWasPressed = false;
                        teamUtil.log("D-Pad RIGHT Was Bumped");
                        if (gamepad1.right_bumper == true) {
                            robot.drive.MIN_END_VELOCITY += 10;
                        } else if (gamepad1.left_bumper == true) {
                            robot.drive.MIN_END_VELOCITY -= 10;
                        }
                    }
                }
                /*
                if (gamepad1.y == true) {
                    robot.drive.runMotors(robot.drive.MIN_START_VELOCITY);
                    teamUtil.pause(1000);
                    robot.drive.runMotors(robot.drive.MIN_END_VELOCITY);
                    teamUtil.pause(1000);
                    robot.drive.runMotors(0);
                }


                //emergency stop
                if (gamepad1.b == true) {
                    robot.drive.runMotors(0);
                }

             */

            /*
                String currentAcceleration = String.format("%d", robot.drive.MAX_ACCELERATION);
                String currentDeceleration = String.format("%d", robot.drive.MAX_DECELERATION);
                String currentMinStartVelocity = String.format("%d", robot.drive.MIN_START_VELOCITY);
                String currentMinEndVelocity = String.format("%d", robot.drive.MIN_END_VELOCITY);
                telemetry.addLine("Current Max Acceleration" + currentAcceleration);
                telemetry.addLine("Current Max Deceleration" + currentDeceleration);
                telemetry.addLine("Current Min Start Velocity" + currentMinStartVelocity);
                telemetry.addLine("Current Min End Velocity" + currentMinEndVelocity);

             */
                robot.outputTelemetry();
                telemetry.update();
            }
        }


}


