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
        drive.setTargetPositionToleranceAllMotors(20);
        outake.runToLevelNoWait(3);


        if(left) {
            drive.strafeRight(.6,140);
            drive.spinLeftToHeading(225, .6);
        }else{
            drive.strafeLeft(.6,155);
            drive.strafeRight(.6, 13);
            drive.spinRightToHeading(127, .6);
        }


        drive.backCM(.6, 12);

        outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
        outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
        teamUtil.pause(100);
        outake.openGrabber();
        teamUtil.pause(500);
        drive.moveCM(.4, 10);
        outake.runToBottomNoWait(true, false);
        if(left) {
            drive.spinRightToHeading(180, .6);
        }else{
            drive.spinLeftToHeading(180, 0.6);
        }
        drive.moveCM(.6, 60);
        if(left) {
            drive.strafeLeftToLine(100, 0.1);
        }else{
            drive.strafeRightToLine(100, 0.1);
        }
        drive.setAllMotorsToSpeed(0.3);
        teamUtil.pause(1000);
        drive.setAllMotorsToSpeed(0);
        outake.closeGrabber();
        teamUtil.pause(500);
        outake.runToLevelNoWait(3);
        drive.backCM(.4, 70);
        if(left) {
            drive.spinLeftToHeading(225, .6);
        }else{
            drive.spinRightToHeading(125, .6);
        }
        drive.backCM(.4, 14);
        outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
        outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
        teamUtil.pause(100);
        outake.openGrabber();
        teamUtil.pause(500);
        drive.moveCM(.4, 11);
        outake.runToBottomNoWait(true, false);
        if(left) {
            drive.spinRightToHeading(180, .6);
        }else{
            drive.spinLeftToHeading(180, 0.6);
        }
        drive.moveCM(.6, 60);
        if(left) {
            drive.strafeLeftToLine(100, 0.1);
        }else{
            drive.strafeRightToLine(100,0.1);
        }
        drive.setAllMotorsToSpeed(0.3);
        teamUtil.pause(1000);
        drive.setAllMotorsToSpeed(0);
        outake.closeGrabber();
        teamUtil.pause(500);
        outake.runToLevelNoWait(3);
        drive.backCM(.4, 70);
        if(left) {
            drive.spinLeftToHeading(225, .6);
        }else{
            drive.spinRightToHeading(125, 0.6);
        }
        drive.backCM(.4, 14);
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
            if(left) {
                drive.moveCM(.6, 60);
            }else{
                drive.backCM(.6, 60);
            }
        }else if(detection == 2){

        }else{
            if(left) {
                drive.backCM(.6, 60);
            }else{
                drive.moveCM(.6, 60);
            }
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
    public void autoV5(boolean left, int detection){

        drive.setHeading(180);
        drive.setTargetPositionToleranceAllMotors(20);
        long startingTime = System.currentTimeMillis();
        outake.runToLevelNoWait(3);


        if(left) {
            drive.strafeRight(.6,140);
            drive.spinLeftToHeading(225, .6);
        }else{
            drive.strafeLeft(.6,140);
            drive.spinRightToHeading(130, .6);

        }


        drive.backCM(.6, 14);

        outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
        outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
        teamUtil.pause(100);
        outake.openGrabber();
        teamUtil.pause(500);
        if(left){
            drive.moveCM(.6, 21);
        }
        else{
            drive.moveCM(.6, 18);
        }

        outake.runToBottomNoWait(true, false);
        if(left) {
            drive.spinRightToHeading(180, .6);
        }else{
            drive.spinLeftToHeading(180, 0.6);
        }
        drive.moveCM(.8, 58);
        /*
        if(left) {
            drive.strafeLeftToLine(100, 0.1);
        }else{
            drive.strafeRightToLine(100, 0.1);
        }

         */

        for(int i=0;i<3; i++){
            drive.setAllMotorsToSpeed(0.3);
            teamUtil.pause(500);
            drive.setAllMotorsToSpeed(0);
            outake.closeGrabber();
            teamUtil.pause(300);
            outake.runToLevelNoWait(3);
            drive.backCM(.4, 60);
            if(left) {
                drive.spinLeftToHeading(220, .6);
            }else{
                drive.spinRightToHeading(132, .6);
            }
            drive.backCM(.6, 22);
            outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
            outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
            teamUtil.pause(100);
            outake.openGrabber();
            teamUtil.pause(200);
            if(left){
                drive.moveCM(.6, 30);
            }
            else{
                drive.moveCM(.6, 22);
            }

            outake.runToBottomNoWait(true, false);
            if(left) {
                drive.spinRightToHeading(180, .6);
            }else{
                drive.spinLeftToHeading(180, 0.6);
            }
            long currentTime = System.currentTimeMillis();

            //time failsafe code
            if(currentTime-startingTime>22000&&detection!=2){
                break;
            }
            else if(currentTime-startingTime>22500){
                break;
            }
            else{

            }

            if(i<2){
                drive.moveCM(.9, 54);
            }
            else{
            }
        }
        /*

        drive.setAllMotorsToSpeed(0.3);
        teamUtil.pause(1000);
        drive.setAllMotorsToSpeed(0);
        outake.closeGrabber();
        teamUtil.pause(500);
        outake.runToLevelNoWait(3);
        drive.backCM(.4, 65);
        if(left) {
            drive.spinLeftToHeading(225, .6);
        }else{
            drive.spinRightToHeading(125, .6);
        }
        drive.backCM(.4, 14);
        outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
        outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
        teamUtil.pause(100);
        outake.openGrabber();
        teamUtil.pause(500);
        drive.moveCM(.6, 14);
        outake.runToBottomNoWait(true, false);
        if(left) {
            drive.spinRightToHeading(180, .6);
        }else{
            drive.spinLeftToHeading(180, 0.6);
        }
        drive.moveCM(.6, 60);

        if(left) {
            drive.strafeLeftToLine(100, 0.1);
        }else{
            drive.strafeRightToLine(100,0.1);
        }


        drive.setAllMotorsToSpeed(0.3);
        teamUtil.pause(1000);
        drive.setAllMotorsToSpeed(0);
        outake.closeGrabber();
        teamUtil.pause(500);
        outake.runToLevelNoWait(3);
        drive.backCM(.6, 70);
        if(left) {
            drive.spinLeftToHeading(225, .6);
        }else{
            drive.spinRightToHeading(125, 0.6);
        }
        drive.backCM(.6, 14);
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

         */

        if(detection == 1){
            if(left) {
                drive.moveCM(.6, 64);
            }else{
                drive.backCM(.6, 64);
            }
        }else if(detection == 2){
            drive.backCM(.6, 12);
        }else{
            if(left) {
                drive.backCM(.6, 70);
            }else{
                drive.moveCM(.6, 60);
            }
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

    public void newAutoV5(boolean left, int detection){

        drive.setHeading(270);

        drive.setTargetPositionToleranceAllMotors(20);
        long startingTime = System.currentTimeMillis();
        //outake.runToLevelNoWait(3);


        if(left) {
            drive.newBackCM(1000,128);
            drive.spinRightToHeading(222, .6);
        }else{
            drive.strafeLeft(.6,155);
            drive.strafeRight(.6, 13);
            drive.spinRightToHeading(127, .6);
        }


        drive.newBackCM(1000, 17);
        /*

        outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
        outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
        teamUtil.pause(100);
        outake.openGrabber();
        teamUtil.pause(500);
        drive.moveCM(.6, 17);
        outake.runToBottomNoWait(true, false);
        if(left) {
            drive.spinRightToHeading(180, .6);
        }else{
            drive.spinLeftToHeading(180, 0.6);
        }
        drive.moveCM(.8, 60);

        if(left) {
            drive.strafeLeftToLine(100, 0.1);
        }else{
            drive.strafeRightToLine(100, 0.1);
        }



        for(int i=0;i<3; i++){
            drive.setAllMotorsToSpeed(0.3);
            teamUtil.pause(500);
            drive.setAllMotorsToSpeed(0);
            outake.closeGrabber();
            teamUtil.pause(300);
            outake.runToLevelNoWait(3);
            drive.backCM(.4, 68);
            if(left) {
                drive.spinLeftToHeading(220, .6);
            }else{
                drive.spinRightToHeading(125, .6);
            }
            drive.backCM(.6, 14);
            outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
            outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
            teamUtil.pause(100);
            outake.openGrabber();
            teamUtil.pause(200);
            drive.moveCM(.6, 19.5);
            outake.runToBottomNoWait(true, false);
            if(left) {
                drive.spinRightToHeading(180, .6);
            }else{
                drive.spinLeftToHeading(180, 0.6);
            }
            long currentTime = System.currentTimeMillis();

            //time failsafe code
            if(currentTime-startingTime>22000&&detection!=2){
                break;
            }
            else if(currentTime-startingTime>22500){
                break;
            }
            else{

            }

            if(i<2){
                drive.moveCM(.9, 60);
            }
            else{
            }
        }


        drive.setAllMotorsToSpeed(0.3);
        teamUtil.pause(1000);
        drive.setAllMotorsToSpeed(0);
        outake.closeGrabber();
        teamUtil.pause(500);
        outake.runToLevelNoWait(3);
        drive.backCM(.4, 65);
        if(left) {
            drive.spinLeftToHeading(225, .6);
        }else{
            drive.spinRightToHeading(125, .6);
        }
        drive.backCM(.4, 14);
        outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
        outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
        teamUtil.pause(100);
        outake.openGrabber();
        teamUtil.pause(500);
        drive.moveCM(.6, 14);
        outake.runToBottomNoWait(true, false);
        if(left) {
            drive.spinRightToHeading(180, .6);
        }else{
            drive.spinLeftToHeading(180, 0.6);
        }
        drive.moveCM(.6, 60);

        if(left) {
            drive.strafeLeftToLine(100, 0.1);
        }else{
            drive.strafeRightToLine(100,0.1);
        }


        drive.setAllMotorsToSpeed(0.3);
        teamUtil.pause(1000);
        drive.setAllMotorsToSpeed(0);
        outake.closeGrabber();
        teamUtil.pause(500);
        outake.runToLevelNoWait(3);
        drive.backCM(.6, 70);
        if(left) {
            drive.spinLeftToHeading(225, .6);
        }else{
            drive.spinRightToHeading(125, 0.6);
        }
        drive.backCM(.6, 14);
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
            if(left) {
                drive.moveCM(.6, 60);
            }else{
                drive.backCM(.6, 60);
            }
        }else if(detection == 2){

        }else{
            if(left) {
                drive.backCM(.6, 60);
            }else{
                drive.moveCM(.6, 60);
            }
        }

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
        // bug in auto right with spin to heading at end
        if(drive.getHeading()>90){
            drive.spinRightToHeading(180,0.6);
        }
        else{
            drive.spinLeftToHeading(180,0.6);
        }

         */

    }
}
