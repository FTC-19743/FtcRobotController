package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

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

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
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

    //if distance is moved exceeds certain threshold
    public void goToWall(int milliseconds, int velocity){
        //system clock (now)\
        //absolute value of current encoder - original position
        long startTime = System.currentTimeMillis();
        //outake.runToBottomNoWait(true,false);
        drive.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.frontLeft.setVelocity(Math.abs(velocity));
        drive.frontRight.setVelocity(Math.abs(velocity));
        drive.backLeft.setVelocity(Math.abs(velocity));
        drive.backRight.setVelocity(Math.abs(velocity));
        //current encoder of certain wheel
        int ticks = drive.frontLeft.getCurrentPosition();
        while(true){
            if(Math.abs(drive.frontLeft.getCurrentPosition()-ticks)>18*drive.COUNTS_PER_CENTIMETER){

                break;
            }
        }
        while(System.currentTimeMillis()-startTime<milliseconds){

        }
        drive.frontLeft.setVelocity(0);
        drive.frontRight.setVelocity(0);
        drive.backLeft.setVelocity(0);
        drive.backRight.setVelocity(0);
        return;
    }

    public void goToWallV2(){
        long opTime = System.currentTimeMillis();

        drive.setAllMotorsRunUsingEncoder();
        FourWheelDrive.MotorData start = new FourWheelDrive.MotorData();
        drive.getDriveMotorData(start);
        teamUtil.log("BACK UP AND TURN");
        while (drive.getEncoderDistance(start) < 13*drive.COUNTS_PER_CENTIMETER) {
            drive.driveMotorsHeadingsFR(225, 180,1500);
        }
        teamUtil.log("DRIVE TOWARDS WALL");
        drive.getDriveMotorData(start);
        while (drive.getEncoderDistance(start) < 29*drive.COUNTS_PER_CENTIMETER) {
            drive.driveMotorsHeadingsFR(180 , 180,1500);
        }
        teamUtil.log("DRIFT TOWARDS LINE");

        drive.getDriveMotorData(start);
        while (!drive.colorSensor.isOnTape() && drive.getEncoderDistance(start) < 30*drive.COUNTS_PER_CENTIMETER) {
            drive.driveMotorsHeadingsFR(135 , 180,500);
        }
        if (drive.colorSensor.isOnTape()) {
            teamUtil.log("FOUND TAPE");
        } else {
            teamUtil.log("FAIL SAFE");
        }

        long startTime = System.currentTimeMillis();
        teamUtil.log("SQUARE ON WALL");
        while (System.currentTimeMillis()-startTime < 750) {
            drive.driveMotorsHeadingsFR(200 , 180,750); // drift left a bit to compensate for late tape reading
        }
        drive.stopDrive();
        teamUtil.log("OP TIME: "+ (System.currentTimeMillis()-opTime));


    }

    public void goToPole(){
        long opTime = System.currentTimeMillis();

        drive.setAllMotorsRunUsingEncoder();
        FourWheelDrive.MotorData start = new FourWheelDrive.MotorData();
        drive.getDriveMotorData(start);
        teamUtil.log("BACK UP");
        while (drive.getEncoderDistance(start) < 50*drive.COUNTS_PER_CENTIMETER) {
            drive.driveMotorsHeadingsFR(340, 180,1500);
        }
        drive.getDriveMotorData(start);
        teamUtil.log("BACK UP AND TURN");
        while (drive.getEncoderDistance(start) < 20*drive.COUNTS_PER_CENTIMETER) {
            drive.driveMotorsHeadingsFR(340, 242,1000);
        }
        drive.getDriveMotorData(start);
        teamUtil.log("BACKUP");
        while (drive.getEncoderDistance(start) < 10*drive.COUNTS_PER_CENTIMETER) {
            drive.driveMotorsHeadingsFR(62, 242,500);
        }
        drive.setAllMotorsActiveBreak();
        teamUtil.log("OP TIME: "+ (System.currentTimeMillis()-opTime));
        teamUtil.pause(1000);
        drive.setAllMotorsRunUsingEncoder();
    }

    public void diagonalStrafeTest(){
        drive.strafeRightDiagonallyToLine(1000,0.6);
        drive.frontLeft.setTargetPosition(drive.frontLeft.getCurrentPosition());
        drive.frontRight.setTargetPosition(drive.frontRight.getCurrentPosition());
        drive.backRight.setTargetPosition(drive.backRight.getCurrentPosition());
        drive.backLeft.setTargetPosition(drive.backLeft.getCurrentPosition());
    }

    public void halfwayJointTest(){
        outake.runToLevelHalfwayJoint(3);
        teamUtil.pause(60000);
    }
    public void runToWallTest(){
        drive.moveCMNoStop(35,1000);
        drive.strafeRightDiagonallyToLine(1000,0.5);
        drive.setAllMotorsToSpeed(0.3);
        teamUtil.pause(1000);
        drive.setAllMotorsToSpeed(0);
    }

    public void autoV6(boolean left, int detection){

        drive.setHeading(270);

        drive.setTargetPositionToleranceAllMotors(20);
        long startingTime = System.currentTimeMillis();
        //outake.runToLevelNoWait(3);


        if(left) {
            drive.newBackCM(1000,90);
            drive.spinRightToHeading(242, .6);
        }else{
            drive.strafeLeft(.6,155);
            drive.strafeRight(.6, 13);
            drive.spinRightToHeading(127, .6);
        }


        drive.newBackCM(1000, 37);
    }

    public void newAutoV5(boolean left, int detection){

        drive.setHeading(270);

        drive.setTargetPositionToleranceAllMotors(20);
        long startingTime = System.currentTimeMillis();
        //outake.runToLevelNoWait(3);


        if(left) {
            drive.newBackCM(1000,90);
            drive.spinRightToHeading(242, .6);
        }else{
            drive.strafeLeft(.6,155);
            drive.strafeRight(.6, 13);
            drive.spinRightToHeading(127, .6);
        }


        drive.newBackCM(1000, 37);

        teamUtil.pause(2000);

        /*
        outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
        outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
        teamUtil.pause(100);
        outake.openGrabber();
        teamUtil.pause(500);
        outake.runToLevel(2);
        outake.runToBottomNoWait(true,false);

         */

        log("Heading: " + drive.getHeading());
        //outake.runToBottomNoWait(true, false);
        goToWallV2();
        if(true){
            return;
        }
        drive.newMoveCM(1000,4);

        drive.spinRightToHeading(180,.5);




        for(int i=0;i<3; i++){
            teamUtil.pause(250);
            goToWall(2500,1000);
            /*
            if(i==0){
                drive.newMoveCM(1000,35);
            }
            else{
                drive.newMoveCM(1000,10);
            }

             */


            outake.closeGrabber();
            teamUtil.pause(300);
            outake.runToLevelHalfwayJointNoWait(3);
            drive.newBackCM(1000, 68);
            if(left) {
                drive.spinLeftToHeading(220, .6);
            }else{
                drive.spinRightToHeading(125, .6);
            }
            teamUtil.pause(60000);
            outake.runToLevel(3);
            drive.backCM(.6, 14);
            outake.pulleyLeft.setTargetPosition(outake.pulleyLeft.getCurrentPosition()-100);
            outake.pulleyRight.setTargetPosition(outake.pulleyRight.getCurrentPosition()-100);
            teamUtil.pause(100);
            outake.openGrabber();
            teamUtil.pause(200);
            outake.runToLevel(2);
            outake.runToBottomNoWait(true, false);
            drive.moveCM(.6, 19.5);

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
