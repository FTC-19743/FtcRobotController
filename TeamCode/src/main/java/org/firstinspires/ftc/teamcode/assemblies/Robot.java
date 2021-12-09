package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class Robot {
    public BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public TwoWheelDrive drive;
    public OutakeArm outakeArm;
    public CarouselSpinner spinner;
    public TSEDetector detector;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Robot() {
        teamUtil.log("Constructing Robot");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new TwoWheelDrive();
        outakeArm = new OutakeArm();
        spinner = new CarouselSpinner();
        detector = new TSEDetector();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        teamUtil.log("Initializing Robot");
        drive.initialize();
        outakeArm.init();
        outakeArm.resetArm();
        spinner.init();
        detector.initialize();
    }
    
    public void doAuto(int path) {
        if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
            if (path != 0) {
                outakeArm.spinnerOutput();
                teamUtil.pause(100);
                outakeArm.spinnerStop();
                drive.moveInches(.3, 36);
                //teamUtil.pause(500);
                if (path == 1) {
                    //outakeArm.runArmToPosition(outakeArm.Level1);
                } else if (path == 2) {
                    outakeArm.runArmToPosition(outakeArm.Level2);
                } else {
                    outakeArm.runArmToPosition(outakeArm.Level3);
                }

                drive.spinLeftWithIMUV2(90, .1);

                if (path == 1) {
                    drive.moveBackInches(.5,2);
                    outakeArm.runArmToPosition(outakeArm.Level1);
                    teamUtil.pause(2000);
                    drive.moveInches(.25, 2);
                } else if (path == 2) {
                    drive.moveInches(.25, 5);
                } else {
                    drive.moveInches(.25, 6.9);
                }
                outakeArm.spinnerOutput();
                teamUtil.pause(1000);
                outakeArm.spinnerStop();


                outakeArm.runToTop();
                if (path == 1) {
                    drive.moveBackInches(.4, 25.5);
                    //drive.moveBackInches(.4, 35);
                } else if (path == 2) {
                    drive.moveBackInches(.4, 29.5);
                } else {
                    drive.moveBackInches(.4, 34.5);
                }

                teamUtil.pause(1000);
                drive.moveInches(.25, 5);
                drive.spinLeftWithIMUV2(90, .1);
                drive.moveInches(.35, 28);

                teamUtil.pause(1000);
                drive.spinLeftWithIMUV2(32, .1);
                drive.moveInches(.35, 12);
                drive.motorsOn(.05);
                spinner.on(.55);
                teamUtil.pause(3600);
                spinner.off();
                drive.motorsOff();
                drive.moveBackInches(.35, 2);
                drive.spinRightWithIMUV2(40, .25);

                drive.moveBackInches(.35, 19.5);

            }
        }
            //code for red alliance
            else {
                if (path != 0) {
                    outakeArm.spinnerOutput();
                    teamUtil.pause(100);
                    outakeArm.spinnerStop();
                    drive.moveInches(.3, 36);
                    //teamUtil.pause(500);
                    if (path == 1) {
                       // outakeArm.runArmToPosition(outakeArm.Level1);
                    } else if (path == 2) {
                        outakeArm.runArmToPosition(outakeArm.Level2);
                    } else {
                        outakeArm.runArmToPosition(outakeArm.Level3);
                    }

                    drive.spinRightWithIMUV2(90, .1);

                    if (path == 1) {
                        drive.moveBackInches(.5,2);
                        outakeArm.runArmToPosition(outakeArm.Level1);
                        teamUtil.pause(2000);
                        drive.moveInches(.25, 2);
                    } else if (path == 2) {
                        drive.moveInches(.25, 5);
                    } else {
                        drive.moveInches(.25, 6.5);
                    }
                    outakeArm.spinnerOutput();
                    teamUtil.pause(1000);
                    outakeArm.spinnerStop();


                    outakeArm.runToTop();
                    if (path == 1) {
                        drive.moveBackInches(.4, 25.5);
                    } else if (path == 2) {
                        drive.moveBackInches(.4, 29.5);
                    } else {
                        drive.moveBackInches(.4, 34.5);
                    }

                    teamUtil.pause(1000);
                    drive.moveInches(.25, 5);
                    drive.spinRightWithIMUV2(90, .1);
                    drive.moveInches(.35, 28);

                    teamUtil.pause(1000);
                    drive.spinRightWithIMUV2(45, .1);
                    drive.moveInches(.35, 11);
                    drive.motorsOn(.05);
                    spinner.on(.55);
                    teamUtil.pause(3100);
                    spinner.off();
                    drive.motorsOff();
                    drive.spinLeftWithIMUV2(45, .25);
                    drive.moveBackInches(.35, 17);

                }


            }




    }
}
