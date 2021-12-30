package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.OutakeArm;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.TwoWheelDrive;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name="TeleopModeBlue", group="Linear Opmode")
public class TeleopModeBlue extends LinearOpMode {
    public BNO055IMU imu;


    Robot robot;
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    double getIMUHeading() {
        Orientation anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (anglesCurrent.firstAngle);
    }

    @Override
    public void runOpMode() {
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        teamUtil.telemetry.addLine("Initializing Op Mode...please wait");
        teamUtil.telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        teamUtil.log("Initializing Drive - FINISHED");


        robot = new Robot();
        robot.init();
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            //gamepad1 will be for the drive
            //gamepad2 will be for the mechanisms
            //blue is gamepad 1
            //red is gamepad 2
            String currentIMU = String.format("%.2f", getIMUHeading());

            //log(currentIMU);

            if(gamepad1.right_bumper == true){
                robot.drive.manualControl(-gamepad1.left_stick_y, gamepad1.right_stick_x);
            } else{
                robot.drive.manualControl(-gamepad1.left_stick_y*2/3, gamepad1.right_stick_x*2/3);
            }

            if(gamepad2.left_trigger > 0){
                robot.outakeArm.spinnerIntake();
            } else if(gamepad2.right_trigger>0){
                robot.outakeArm.spinnerOutput();
            } else {
                robot.outakeArm.spinnerStop();
            }
            if(gamepad2.dpad_down==true){
                //while(gamepad2.dpad_down){ }
                robot.outakeArm.runArmToPosition(robot.outakeArm.Level1);
            } else if(gamepad2.dpad_left==true||gamepad2.dpad_right==true){
                //while(gamepad2.dpad_left||gamepad2.dpad_right){ }
                robot.outakeArm.runArmToPosition(robot.outakeArm.Level2);
            } else if(gamepad2.dpad_up==true){
                //while(gamepad2.dpad_up){ }
                robot.outakeArm.runArmToPosition(robot.outakeArm.Level3);
            } else if(gamepad2.a==true){
                //while(gamepad2.a){ }
                robot.outakeArm.runArmToPosition(robot.outakeArm.Ground);

            } else if(gamepad2.y == true) {
                //while(gamepad2.y){ }
                robot.outakeArm.runArmToPosition(robot.outakeArm.Top);
            }

            //else if(gamepad2.y==true){
            //while(gamepad2.y){
            // }

            // robot.outakeArm.resetArm();

            // robot.drive.manualControl(0,0);
            // }
            if(gamepad2.right_bumper==true) {
                robot.spinner.on(-.75);
            }
            else{
                robot.spinner.off();
            }
            robot.outakeArm.writeTelemetry();
            robot.drive.writeTelemetry();
            robot.spinner.spinnerTelemetry();
            robot.outakeSlide.writeTelemetry();
            telemetry.update();


        }
    }
}