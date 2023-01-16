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
    int defaultThreshold = 20;

    public Robot23(){
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new FourWheelDrive();
        outake = new Outake();


    }

    public void initialize(){
        drive.initialize();
        outake.init();

    }

    public void calibrate(){
        outake.calibrate();
        //drive.calibrate();
    }

    public void outputTelemetry(){
        outake.outputTelemetry();
        drive.outputTelemetry();
    }
    public void driveInSquare(){
        //drive.moveCM(.2, 61);

        drive.strafeRight(.2, 61);/*
        drive.backCM(.2, 5);
        drive.strafeLeft(.2, 5);*/
    }
    public void auto(boolean left, int detection){
        drive.setHeading(180);

        outake.runToLevelNoWait(3);


        if(left) {
            drive.strafeRight(.6,140);
            drive.spinLeftToHeading(225, .6);
        }else{
            drive.strafeLeft(.6,145);
            drive.spinRightToHeading(135, .6);
        }
        drive.setTargetPositionToleranceAllMotors(20);


        if(left) {
            drive.backCM(.6, 12);
        }else{
            drive.backCM(.6, 9);
        }

        outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
        outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
        teamUtil.pause(100);
        outake.openGrabber();
        teamUtil.pause(500);
        drive.frontLeft.setTargetPositionTolerance(20);
        drive.frontRight.setTargetPositionTolerance(20);
        drive.backLeft.setTargetPositionTolerance(20);
        drive.backRight.setTargetPositionTolerance(20);
        if(left) {
            drive.moveCM(.4, 11);
        }else{
            drive.moveCM(.4, 10);
        }
        drive.frontLeft.setTargetPositionTolerance(defaultThreshold);
        drive.frontRight.setTargetPositionTolerance(defaultThreshold);
        drive.backLeft.setTargetPositionTolerance(defaultThreshold);
        drive.backRight.setTargetPositionTolerance(defaultThreshold);
        outake.runToBottomNoWait(true, false);
        if(left) {
            drive.spinRightToHeading(180, .6);
        }else{
            drive.spinLeftToHeading(180, 0.6);
        }
        drive.moveCM(.6, 52);
        drive.strafeLeftToLine(100, 0.1);
        drive.setAllMotorsToSpeed(0.3);
        teamUtil.pause(1000);
        drive.setAllMotorsToSpeed(0);
        outake.closeGrabber();
        teamUtil.pause(500);
        outake.runToLevelNoWait(3);
        drive.backCM(.4, 70);
        drive.spinLeftToHeading(225, .3);
        drive.backCM(.4, 14);
        drive.frontLeft.setTargetPositionTolerance(defaultThreshold);
        drive.frontRight.setTargetPositionTolerance(defaultThreshold);
        drive.backLeft.setTargetPositionTolerance(defaultThreshold);
        drive.backRight.setTargetPositionTolerance(defaultThreshold);
        outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
        outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
        teamUtil.pause(100);
        outake.openGrabber();
        teamUtil.pause(500);
        if(left) {
            drive.moveCM(.4, 11);
        }else{
            drive.moveCM(.4, 10);
        }
        drive.frontLeft.setTargetPositionTolerance(defaultThreshold);
        drive.frontRight.setTargetPositionTolerance(defaultThreshold);
        drive.backLeft.setTargetPositionTolerance(defaultThreshold);
        drive.backRight.setTargetPositionTolerance(defaultThreshold);
        outake.runToBottomNoWait(true, false);
        if(left) {
            drive.spinRightToHeading(180, .6);
        }else{
            drive.spinLeftToHeading(180, 0.6);
        }
        drive.moveCM(.6, 60);
        drive.strafeLeftToLine(100, 0.1);
        drive.setAllMotorsToSpeed(0.3);
        teamUtil.pause(1000);
        drive.setAllMotorsToSpeed(0);
        outake.closeGrabber();
        teamUtil.pause(500);
        outake.runToLevelNoWait(3);
        drive.backCM(.4, 70);
        drive.spinLeftToHeading(225, .3);
        drive.backCM(.4, 14);
        drive.frontLeft.setTargetPositionTolerance(defaultThreshold);
        drive.frontRight.setTargetPositionTolerance(defaultThreshold);
        drive.backLeft.setTargetPositionTolerance(defaultThreshold);
        drive.backRight.setTargetPositionTolerance(defaultThreshold);
        outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
        outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
        teamUtil.pause(100);
        outake.openGrabber();
        teamUtil.pause(500);

        drive.moveCM(.6,15);
        outake.runToBottomNoWait(false, false);


        if(left){
            drive.spinRightToHeading(180,0.6);
        }
        else{
            drive.spinLeftToHeading(180,0.6);
        }

        if(detection == 1){
            drive.moveCM(.6,60);
        }else if(detection == 2){

        }else{
            drive.backCM(.6,60);
        }
        /*
        for(int i=0;i<3; i++){

            drive.frontLeft.setTargetPositionTolerance(20);
            drive.frontRight.setTargetPositionTolerance(20);
            drive.backLeft.setTargetPositionTolerance(20);
            drive.backRight.setTargetPositionTolerance(20);
            if(left) {
                drive.moveCM(.4, 12);
            }else{
                drive.moveCM(.4, 10);
            }
            drive.frontLeft.setTargetPositionTolerance(defaultThreshold);
            drive.frontRight.setTargetPositionTolerance(defaultThreshold);
            drive.backLeft.setTargetPositionTolerance(defaultThreshold);
            drive.backRight.setTargetPositionTolerance(defaultThreshold);
            outake.runToBottomNoWait(true, false);
            if(left) {
                drive.spinRightToHeading(180, .3);
            }else{
                drive.spinLeftToHeading(180, 0.3);
            }
            drive.frontLeft.setTargetPositionTolerance(20);
            drive.frontRight.setTargetPositionTolerance(20);
            drive.backLeft.setTargetPositionTolerance(20);
            drive.backRight.setTargetPositionTolerance(20);
            if(left&&i==0) {
                drive.moveCM(.6, 50);
            }else if (!left && i==0){
                drive.moveCM(.6, 52);
            }
            drive.frontLeft.setTargetPositionTolerance(defaultThreshold);
            drive.frontRight.setTargetPositionTolerance(defaultThreshold);
            drive.backLeft.setTargetPositionTolerance(defaultThreshold);
            drive.backRight.setTargetPositionTolerance(defaultThreshold);
            if(i == 0) {
                if (left) {
                    drive.strafeLeftToLine(100, 0.1);
                } else {
                    drive.strafeRightToLine(100, 0.1);
                }
                drive.setAllMotorsToSpeed(0.3);
                teamUtil.pause(1000);
                drive.setAllMotorsToSpeed(0);
            } else{
                drive.setAllMotorsToSpeed(0.3);
                teamUtil.pause(2000);
                drive.setAllMotorsToSpeed(0);
            }



            outake.closeGrabber();
            teamUtil.pause(500);
            outake.runToLevelNoWait(3);
            drive.backCM(.4, 70);
            if(left) {
                drive.spinLeftToHeading(223, .3);
            }else{
                drive.spinRightToHeading(127, .3);
            }
            drive.frontLeft.setTargetPositionTolerance(20);
            drive.frontRight.setTargetPositionTolerance(20);
            drive.backLeft.setTargetPositionTolerance(20);
            drive.backRight.setTargetPositionTolerance(20);
            if(left) {
                drive.backCM(.4, 14);
            }else{
                drive.backCM(.4, 10); // was 13 for both
            }
            drive.frontLeft.setTargetPositionTolerance(defaultThreshold);
            drive.frontRight.setTargetPositionTolerance(defaultThreshold);
            drive.backLeft.setTargetPositionTolerance(defaultThreshold);
            drive.backRight.setTargetPositionTolerance(defaultThreshold);
            outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
            outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
            teamUtil.pause(100);
            outake.openGrabber();
            teamUtil.pause(500);
        }
        drive.frontLeft.setTargetPositionTolerance(20);
        drive.frontRight.setTargetPositionTolerance(20);
        drive.backLeft.setTargetPositionTolerance(20);
        drive.backRight.setTargetPositionTolerance(20);
        drive.moveCM(.6,15);
        outake.runToBottomNoWait(true, false);


        if(left){
            drive.spinRightToHeading(180,0.6);
        }
        else{
            drive.spinLeftToHeading(180,0.6);
        }

        if(detection == 1){
            drive.moveCM(.6,60);
        }else if(detection == 2){

        }else{
            drive.backCM(.6,60);
        }
        //bug in auto right with spin to heading at end
        if(drive.getHeading()>90){
            drive.spinRightToHeading(180,0.6);
        }
        else{
            drive.spinLeftToHeading(180,0.6);
        }

         */
    }
}
