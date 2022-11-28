package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class Robot23 {
    public BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public FourWheelDrive drive;
    public Outake outake;
    public bottomColorSensor colorSensor;
    public ColorSensor ftcColorSensor;

    public Robot23(){
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new FourWheelDrive();
        outake = new Outake();
        colorSensor = new bottomColorSensor(hardwareMap.get(ColorSensor.class, "bottomColor"));


    }

    public void initialize(){
        drive.initialize();
        outake.init();

    }

    public void calibrate(){
        outake.calibrate();
        colorSensor.calibrate();

    }

    public void outputTelemetry(){
        outake.outputTelemetry();
        drive.outputTelemetry();
        telemetry.addData("Is On Line", "%b", colorSensor.isOnTape());
    }
    public void driveInSquare(){
        //drive.moveCM(.2, 61);

        drive.strafeRight(.2, 61);/*
        drive.backCM(.2, 5);
        drive.strafeLeft(.2, 5);*/
    }
}
