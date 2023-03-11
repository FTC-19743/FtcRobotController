package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.teamcode.assemblies.FourWheelDrive;
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
        robot.drive.setTargetPositionToleranceAllMotors(20);
        robot.drive.setNewPIDCoefficients();
        PIDFCoefficients frontLeftPID = robot.drive.frontLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        log("Front Left PID" + frontLeftPID);
        double powerFactor = 1;

        while(opModeIsActive()){

            driverGamepad.loop();
            armsGamepad.loop();

            //declaration of power and denominator variables for math
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;
            double denominator;

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x*1.1 ; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            if(Math.abs(rx)<0.15){
                rx=0;
            }
            //Orientation anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double botHeading = -Math.toRadians((robot.drive.getHeading()-180)); //
            //double botHeading = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle; //removed negative
            telemetry.addLine("botheading: " + Math.toDegrees(botHeading));
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            //telemetry.addLine("current cup level "+ String.valueOf(cupLevel));


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]



            //working code for robot centric drive
            if(gamepad1.left_bumper) {
                //working robot centric
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;
            }
            else{
                //working field centric
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;
            }




            if (gamepad1.right_trigger > .8) {
                powerFactor = 1;
            }else if(gamepad1.left_trigger>.8){
                powerFactor=0.25;
            }
            else{
                powerFactor = .5;
            }
            robot.drive.frontLeft.setPower(-frontLeftPower*powerFactor);
            robot.drive.backLeft.setPower(-backLeftPower*powerFactor);
            robot.drive.frontRight.setPower(-frontRightPower*powerFactor);
            robot.drive.backRight.setPower(-backRightPower*powerFactor);

            if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
                robot.drive.setHeading(180);
            }

            if(driverGamepad.wasAPressed()){
                robot.drive.setHeading(238);
                robot.goToWallV2();
            }

            if(driverGamepad.wasDownPressed()){
                robot.outake.runToBottom(false,false);
            }

            if(driverGamepad.wasLeftPressed()){
                robot.drive.setHeading(238);
                robot.park(1,0);
            }
            if(driverGamepad.wasRightPressed()){
                robot.drive.setHeading(238);
                robot.park(2,0);
            }
            if(driverGamepad.wasUpPressed()){
                robot.drive.setHeading(270);
                robot.firstRunToPoleLeftV2();
            }

            if(driverGamepad.wasYPressed()){
                robot.drive.setHeading(180);
                robot.goToPoleV2();

            }
            if (driverGamepad.wasXPressed()) { // Go to Wall
                long opTime = System.currentTimeMillis();

                robot.drive.setAllMotorsRunUsingEncoder();
                FourWheelDrive.MotorData start = new FourWheelDrive.MotorData();
                robot.drive.getDriveMotorData(start);
                teamUtil.log("BACK UP AND TURN");
                while (robot.drive.getEncoderDistance(start) < 25*robot.drive.COUNTS_PER_CENTIMETER) {
                    robot.drive.driveMotorsHeadingsFR(225, 180,1500);
                }
                teamUtil.log("DRIVE TOWARDS WALL");
                robot.drive.getDriveMotorData(start);
                while (robot.drive.getEncoderDistance(start) < 30*robot.drive.COUNTS_PER_CENTIMETER) {
                    robot.drive.driveMotorsHeadingsFR(180 , 180,1500);
                }
                teamUtil.log("DRIFT TOWARDS LINE");
                robot.drive.getDriveMotorData(start);
                while (!robot.drive.colorSensor.isOnTape() && robot.drive.getEncoderDistance(start) < 30*robot.drive.COUNTS_PER_CENTIMETER) {
                    robot.drive.driveMotorsHeadingsFR(135 , 180,1000);
                }
                if (robot.drive.colorSensor.isOnTape()) {
                    teamUtil.log("FOUND TAPE");
                } else {
                    teamUtil.log("FAIL SAFE");
                }
                long startTime = System.currentTimeMillis();
                teamUtil.log("SQUARE ON WALL");
                while (System.currentTimeMillis()-startTime < 500) {
                    robot.drive.driveMotorsHeadingsFR(200 , 180,750); // drift left a bit to compensate for late tape reading
                }
                robot.drive.stopDrive();
                teamUtil.log("OP TIME: "+ (System.currentTimeMillis()-opTime));

            }
            if (driverGamepad.wasBPressed()) { // Go To Pole
                long opTime = System.currentTimeMillis();

                robot.drive.setAllMotorsRunUsingEncoder();
                FourWheelDrive.MotorData start = new FourWheelDrive.MotorData();
                robot.drive.getDriveMotorData(start);
                teamUtil.log("BACK UP");
                while (robot.drive.getEncoderDistance(start) < 40*robot.drive.COUNTS_PER_CENTIMETER) {
                    robot.drive.driveMotorsHeadingsFR(350, 180,1500);
                }
                robot.drive.getDriveMotorData(start);
                teamUtil.log("BACK UP AND TURN");
                while (robot.drive.getEncoderDistance(start) < 30*robot.drive.COUNTS_PER_CENTIMETER) {
                    robot.drive.driveMotorsHeadingsFR(355, 225,1000);
                }
                robot.drive.getDriveMotorData(start);
                teamUtil.log("BACKUP");
                while (robot.drive.getEncoderDistance(start) < 10*robot.drive.COUNTS_PER_CENTIMETER) {
                    robot.drive.driveMotorsHeadingsFR(45, 225,500);
                }
                robot.drive.setAllMotorsActiveBreak();
                teamUtil.log("OP TIME: "+ (System.currentTimeMillis()-opTime));
                teamUtil.pause(1000);
                robot.drive.setAllMotorsRunUsingEncoder();
            }


            robot.drive.outputTelemetry();
            telemetry.update();
        }


    }
}
