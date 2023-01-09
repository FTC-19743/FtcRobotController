package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public class FourWheelDrive {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    public bottomColorSensor colorSensor;


    public BNO055IMU imu; //This variable is the imu
    public static double HEADING_OFFSET; // offset between IMU heading and field

    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;


    public double COUNTS_PER_MOTOR_REV = 537.7;    // GoBilda 5202 312 RPM
    public double COUNTS_PER_CENTIMETER = 17.923;
    public double MIN_START_VELOCITY = 200; //tentative value
    public double MIN_END_VELOCITY = 100; //tentative value
    public double MAX_ACCELERATION = 10; //tentative value
    public double MAX_DECELERATION = -5; //tentative value (should be negative)


    public FourWheelDrive() {
        teamUtil.log("Constructing Drive");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
        colorSensor = new bottomColorSensor(hardwareMap.get(ColorSensor.class, "bottomColor"));
    }

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public void initialize() {


        teamUtil.log("Initializing Drive");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = hardwareMap.get(DcMotorEx.class, "flm");
        frontRight = hardwareMap.get(DcMotorEx.class, "frm");
        backLeft = hardwareMap.get(DcMotorEx.class, "blm");
        backRight = hardwareMap.get(DcMotorEx.class, "brm");

        // colorSensor.calibrate();
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //frontLeft.setTargetPositionTolerance(20);
       // frontRight.setTargetPositionTolerance(20);
       // backLeft.setTargetPositionTolerance(20);
        //backRight.setTargetPositionTolerance(20);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        teamUtil.log("Initializing Drive - FINISHED");




    }
    public void calibrate(){
        colorSensor.calibrate();
        teamUtil.log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
    }
    public double getIMUHeading() {
        Orientation anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (anglesCurrent.firstAngle);
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return our current heading as a 0 to 360 range.
    public double getHeading() {
        return adjustAngle(getIMUHeading() - HEADING_OFFSET);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Make the current heading 0.
    public void resetHeading() {
        HEADING_OFFSET = getIMUHeading();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Make the current heading to specified number
    public void setHeading(int heading){
        HEADING_OFFSET = getIMUHeading()-heading;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // adjust the given angle to be in the range 0-360.
    public double adjustAngle(double angle) {
        //assuming imu runs from [0, 360] and angle is added/subtracted, adjust it to expected reading
        while (angle >= 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    /*
    public void updatePowers(){
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }

     */

    public void strafeLeft(double speed, double centimeters) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        centimeters *= 1.11;
        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        teamUtil.log("Strafing left");
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeRight(double speed, double centimeters) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        centimeters *= 1.11;
        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        teamUtil.log("Strafing right");
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void newStrafeRight(double speed, double centimeters){
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.1));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.1));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.1));
        backRight.setTargetPosition((int)(newBackRightTarget*.1));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.4));
        frontRight.setPower(Math.abs(speed)*.4);
        backLeft.setPower(Math.abs(speed)*.4);
        backRight.setPower(Math.abs(speed)*.4);
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.3));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.3));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.3));
        backRight.setTargetPosition((int)(newBackRightTarget*.3));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.7));
        frontRight.setPower(Math.abs(speed)*.7);
        backLeft.setPower(Math.abs(speed)*.7);
        backRight.setPower(Math.abs(speed)*.7);
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.7));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.7));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.7));
        backRight.setTargetPosition((int)(newBackRightTarget*.7));

        // Turn On RUN_TO_POSITION


        // start motion.
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));
        teamUtil.log("Moving Forward");
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget));
        frontRight.setTargetPosition((int)(newFrontRightTarget));
        backLeft.setTargetPosition((int)(newBackLeftTarget));
        backRight.setTargetPosition((int)(newBackRightTarget));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.4));
        frontRight.setPower(Math.abs(speed)*.4);
        backLeft.setPower(Math.abs(speed)*.4);
        backRight.setPower(Math.abs(speed)*.4);
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void newStrafeLeft(double speed, double centimeters){
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() - (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.1));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.1));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.1));
        backRight.setTargetPosition((int)(newBackRightTarget*.1));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.4));
        frontRight.setPower(Math.abs(speed)*.4);
        backLeft.setPower(Math.abs(speed)*.4);
        backRight.setPower(Math.abs(speed)*.4);
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.3));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.3));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.3));
        backRight.setTargetPosition((int)(newBackRightTarget*.3));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.7));
        frontRight.setPower(Math.abs(speed)*.7);
        backLeft.setPower(Math.abs(speed)*.7);
        backRight.setPower(Math.abs(speed)*.7);
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.7));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.7));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.7));
        backRight.setTargetPosition((int)(newBackRightTarget*.7));

        // Turn On RUN_TO_POSITION


        // start motion.
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));
        teamUtil.log("Moving Forward");
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget));
        frontRight.setTargetPosition((int)(newFrontRightTarget));
        backLeft.setTargetPosition((int)(newBackLeftTarget));
        backRight.setTargetPosition((int)(newBackRightTarget));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.4));
        frontRight.setPower(Math.abs(speed)*.4);
        backLeft.setPower(Math.abs(speed)*.4);
        backRight.setPower(Math.abs(speed)*.4);
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void outputTelemetry() {
        telemetry.addData("Output  ", "flm:%d frm:%d blm:%d brm:%d heading:%f nh: %f",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition(), getIMUHeading(), getHeading());

        telemetry.addData("Is On Line", "%b", colorSensor.isOnTape());
        telemetry.addData("Red Value ", colorSensor.redValue());
        telemetry.addData("Blue Value ", colorSensor.blueValue());
        telemetry.addLine("Target Position Tolerance" + frontLeft.getTargetPositionTolerance());

    }

    //basic move centimeters without accel and deceleration
    public void newMoveCM(double speed, double centimeters) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.1));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.1));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.1));
        backRight.setTargetPosition((int)(newBackRightTarget*.1));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.4));
        frontRight.setPower(Math.abs(speed)*.4);
        backLeft.setPower(Math.abs(speed)*.4);
        backRight.setPower(Math.abs(speed)*.4);
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.3));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.3));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.3));
        backRight.setTargetPosition((int)(newBackRightTarget*.3));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.7));
        frontRight.setPower(Math.abs(speed)*.7);
        backLeft.setPower(Math.abs(speed)*.7);
        backRight.setPower(Math.abs(speed)*.7);
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget*.7));
        frontRight.setTargetPosition((int)(newFrontRightTarget*.7));
        backLeft.setTargetPosition((int)(newBackLeftTarget*.7));
        backRight.setTargetPosition((int)(newBackRightTarget*.7));

        // Turn On RUN_TO_POSITION


        // start motion.
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));
        teamUtil.log("Moving Forward");
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setTargetPosition((int)(newFrontLeftTarget));
        frontRight.setTargetPosition((int)(newFrontRightTarget));
        backLeft.setTargetPosition((int)(newBackLeftTarget));
        backRight.setTargetPosition((int)(newBackRightTarget));
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(Math.abs(speed*.4));
        frontRight.setPower(Math.abs(speed)*.4);
        backLeft.setPower(Math.abs(speed)*.4);
        backRight.setPower(Math.abs(speed)*.4);
        currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }
    public void moveCM(double speed, double centimeters) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Determine new target position, and pass to motor controller\
        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (centimeters * COUNTS_PER_CENTIMETER);

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // start motion.
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));
        teamUtil.log("Moving Forward");
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            //teamUtil.log("waiting");
        }

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    public void backCM(double speed, double centimeters) {
        moveCM(speed, -centimeters);
    }

    public void runMotors(double velocity) {
        frontLeft.setVelocity(velocity);
        frontRight.setVelocity(velocity);
        backLeft.setVelocity(velocity);
        backRight.setVelocity(velocity);
    }


    public void moveCmWAcceleration(double cruiseVelocity, double centimeters) {
        double startEncoderPosition = frontLeft.getCurrentPosition();


        double velocityChangeNeededAccel = cruiseVelocity - MIN_START_VELOCITY;
        double velocityChangeNeededDecel = cruiseVelocity - MIN_END_VELOCITY;


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double totalTics = centimeters * COUNTS_PER_CENTIMETER;
        double ticsDuringAcceleration = velocityChangeNeededAccel / MAX_ACCELERATION;
        double ticsDuringDeceleration = velocityChangeNeededDecel / MAX_DECELERATION;
        double cruiseTics = totalTics - ticsDuringDeceleration - ticsDuringAcceleration;

        while (frontLeft.getCurrentPosition() < startEncoderPosition + ticsDuringAcceleration) {
            double ticsSinceStart = frontLeft.getCurrentPosition() - startEncoderPosition;

            runMotors(MAX_ACCELERATION * ticsSinceStart + MIN_START_VELOCITY);
        }

        while (frontLeft.getCurrentPosition() < cruiseTics + startEncoderPosition) {
            runMotors(cruiseVelocity);
        }

        double encoderAfterCruise = frontLeft.getCurrentPosition();

        while (frontLeft.getCurrentPosition() < startEncoderPosition + totalTics) {
            double ticsSinceCruise = frontLeft.getCurrentPosition() - encoderAfterCruise;

            runMotors(MAX_DECELERATION * ticsSinceCruise + MIN_END_VELOCITY);
        }
        runMotors(0);

    }

    public void spinRightWithIMU(double degrees, double speed) {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double initialIMU = getIMUHeading();
        double IMUNeeded = initialIMU - degrees + 8;
        String initialIMUToPrint = String.format("%.2f", initialIMU);
        String IMUNeededToPrint = String.format("%.2f", IMUNeeded);
        log("Initial IMU: " + initialIMUToPrint);
        log("IMU Needed: " + IMUNeededToPrint);
        //if the end IMU is an impossible value, this code allows the robot to transition
        //from the left hemisphere to the right without issues
        //IMU Diagram Doc: https://docs.google.com/document/d/1RI6dZkmHRWhUBy-ZgONwAEO7AxOb_vjcoX40VSjJYjg/edit
        if (IMUNeeded < -180) {
            double currentIMU = getIMUHeading();
            while (currentIMU < 0) {
                currentIMU = getIMUHeading();
                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                frontRight.setPower(-1 * speed);
                backRight.setPower(-1 * speed);
                String currentIMUToPrint = String.format("%.2f", currentIMU);
               // log(currentIMUToPrint);
            }
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            //finds out how many degrees were traveled depending on where the robot was initially
            //facing
            double degreesTraveled = 0;
            if (initialIMU < 0) {
                degreesTraveled = 180 + initialIMU;
            } else if (initialIMU == 0) {

            } else {
                degreesTraveled = 180 - initialIMU;
            }
            currentIMU = getIMUHeading();
            //Finds out how far to travel and prints important values
            double degreesLeft = degrees - degreesTraveled;
            double IMUNeeded2 = 179.999999 - degreesLeft;
            String degreesTraveledToPrint = String.format("%.2f", degreesTraveled);
            log(degreesTraveledToPrint);
            String degreesLeftToPrint = String.format("%.2f", degreesLeft);
            log(degreesLeftToPrint);

            String IMUNeeded2ToPrint = String.format("%.2f", IMUNeeded2);


            log(IMUNeeded2ToPrint);

            while (currentIMU > IMUNeeded2) {
                //currentIMU=getIMUHeading();
                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                frontRight.setPower(-1 * speed);
                backRight.setPower(-1 * speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
                //log(currentIMUToPrint);
                currentIMU = getIMUHeading();

            }
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

        }
        //If math is simple and no conversion is needed the robot will spin without issue
        else {
            double currentIMU = getIMUHeading();
            while (currentIMU > IMUNeeded) {
                //currentIMU=getIMUHeading();
                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                frontRight.setPower(-1 * speed);
                backRight.setPower(-1 * speed);
                String currentIMUToPrint = String.format("%.2f", currentIMU);
                //log(currentIMUToPrint);
                currentIMU = getIMUHeading();
            }
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }


    }

    public void spinLeftWithIMU(double degrees, double speed){
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double initialIMU = getIMUHeading();
        double IMUNeeded = initialIMU+degrees-8;
        String initialIMUToPrint = String.format("%.2f", initialIMU);
        String IMUNeededToPrint = String.format("%.2f", IMUNeeded);
        log("Initial IMU: " + initialIMUToPrint);
        log("IMU Needed: " + IMUNeededToPrint);

        //IMU Diagram Doc: https://docs.google.com/document/d/1RI6dZkmHRWhUBy-ZgONwAEO7AxOb_vjcoX40VSjJYjg/edit
        if(IMUNeeded>180){
            double currentIMU = getIMUHeading();
            while(currentIMU>0){
                currentIMU=getIMUHeading();
                frontLeft.setPower(-1*speed);
                backLeft.setPower(-1*speed);
                frontRight.setPower(speed);
                backRight.setPower(speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
                //log(currentIMUToPrint);
            }
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            //finds out how many degrees were traveled depending on where the robot was initially
            //facing
            double degreesTraveled=0;
            if(initialIMU < 0){
                degreesTraveled = 179.999-initialIMU;
            }
            else if(initialIMU==0){

            }
            else{
                degreesTraveled = 179.999+initialIMU*-1;
            }
            currentIMU=getIMUHeading();
            //Finds out how far to travel and prints important values
            double degreesLeft = degrees-degreesTraveled;
            double IMUNeeded2 = -179.999999+degreesLeft;
            String degreesTraveledToPrint = String.format("%.2f", degreesTraveled);
            String degreesLeftToPrint = String.format("%.2f", degreesLeft);
            String IMUNeeded2ToPrint = String.format("%.2f", IMUNeeded2);
            log("Degrees Traveled: " + degreesTraveledToPrint);
            log("Degrees Left To Travel " + degreesLeftToPrint);
            log("IMU needed as robot enters left hemisphere: " + IMUNeeded2ToPrint);

            while(currentIMU<IMUNeeded2){
                currentIMU=getIMUHeading();
                frontLeft.setPower(-1*speed);
                backLeft.setPower(-1*speed);
                frontRight.setPower(speed);
                backRight.setPower(speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
                //log(currentIMUToPrint);
                currentIMU=getIMUHeading();

            }
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

        }
        //If math is simple and no conversion is needed the robot will spin without issue
        else{
            double currentIMU = getIMUHeading();
            while(currentIMU<IMUNeeded){
                currentIMU=getIMUHeading();
                frontLeft.setPower(-1*speed);
                backLeft.setPower(-1*speed);
                frontRight.setPower(speed);
                backRight.setPower(speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
               // log(currentIMUToPrint);
                currentIMU=getIMUHeading();
            }
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }








    }

    public void strafeRightToLine(){
        log("Strafing Right To Line");
        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(0.1);
        backLeft.setPower(-0.1);
        backRight.setPower(0.1);
        frontRight.setPower(-0.1);
        while(!colorSensor.isOnTape()){
            log("Not On Line");
            //log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" +colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
            //log("On Blue " +colorSensor.onBlue()+ " On Red " +colorSensor.onRed());

        }
        log("On Line");
        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition());
        backLeft.setTargetPosition(backLeft.getCurrentPosition());

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        log("Done Strafing Right To Line");



    }
    public void strafeLeftToLine(){
        log("Strafing Left To Line");
        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(-0.1);
        backLeft.setPower(0.1);
        backRight.setPower(-0.1);
        frontRight.setPower(0.1);
        while(!colorSensor.isOnTape()){
            log("Not On Line");
            //log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" +colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
            //log("On Blue " +colorSensor.onBlue()+ " On Red " +colorSensor.onRed());

        }
        log("On Line");
        log("Blue Value " +colorSensor.blueValue()+ "Blue Threshold" + colorSensor.BLUE_THRESHOLD + " Red Value "+  colorSensor.redValue() + " Red Threshold " + colorSensor.RED_THRESHOLD);
        log("On Blue " +colorSensor.onBlue()+ " On Red " + colorSensor.onRed());
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition());
        backLeft.setTargetPosition(backLeft.getCurrentPosition());

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        log("Done Strafing Right To Line");



    }
    public void relativeSpinRight(double degrees, double topSpeed){
        setHeading(0);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(getHeading()<degrees*.1){
            frontLeft.setPower(-.4*topSpeed);
            backLeft.setPower(-.4*topSpeed);
            frontRight.setPower(.4*topSpeed);
            backRight.setPower(.4*topSpeed);
        }
        while(getHeading()<degrees*.2){
            frontLeft.setPower(-.7*topSpeed);
            backLeft.setPower(-.7*topSpeed);
            frontRight.setPower(.7*topSpeed);
            backRight.setPower(.7*topSpeed);
        }
        while(getHeading()<degrees*.8){
            frontLeft.setPower(-topSpeed);
            backLeft.setPower(-topSpeed);
            frontRight.setPower(topSpeed);
            backRight.setPower(topSpeed);
        }
        while(getHeading()<degrees*.9){
            frontLeft.setPower(-.6*topSpeed);
            backLeft.setPower(-.6*topSpeed);
            frontRight.setPower(.6*topSpeed);
            backRight.setPower(.6*topSpeed);
        }
        while(getHeading()<degrees){
            frontLeft.setPower(-.2*topSpeed);
            backLeft.setPower(-.2*topSpeed);
            frontRight.setPower(.2*topSpeed);
            backRight.setPower(.2*topSpeed);
        }
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public void setAllMotorsToSpeed(double speed){
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
    }

    public void relativeSpinLeft(double degrees, double topSpeed){
        setHeading(0);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(getHeading()<360-degrees*.1){
            frontLeft.setPower(.2*topSpeed);
            backLeft.setPower(.2*topSpeed);
            frontRight.setPower(-.2*topSpeed);
            backRight.setPower(-.2*topSpeed);
        }
        while(getHeading()<360-degrees*.2){
            frontLeft.setPower(.6*topSpeed);
            backLeft.setPower(.6*topSpeed);
            frontRight.setPower(-.6*topSpeed);
            backRight.setPower(-.6*topSpeed);
        }
        while(getHeading()<360-degrees*.7){
            frontLeft.setPower(topSpeed);
            backLeft.setPower(topSpeed);
            frontRight.setPower(-topSpeed);
            backRight.setPower(-topSpeed);
        }
        while(getHeading()<360-degrees*.9){
            frontLeft.setPower(.6*topSpeed);
            backLeft.setPower(.6*topSpeed);
            frontRight.setPower(-.6*topSpeed);
            backRight.setPower(-.6*topSpeed);
        }
        while(getHeading()<360-degrees){
            frontLeft.setPower(.2*topSpeed);
            backLeft.setPower(.2*topSpeed);
            frontRight.setPower(-.2*topSpeed);
            backRight.setPower(-.2*topSpeed);
        }
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public void spinRightToHeading(int wantedIMU, double speed){
        log("Spin Right To Heading Called");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(-speed);
        backRight.setPower(-speed);
        while(getHeading()>(wantedIMU+30)){

        }
        frontLeft.setPower(0.15);
        backLeft.setPower(0.15);
        frontRight.setPower(-0.15);
        backRight.setPower(-0.15);
        while(getHeading()>(wantedIMU+3)){
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);



    }



    public void spinLeftToHeading(int wantedIMU, double speed){
        log("Spin Left To Heading Called");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        while(getHeading()<(wantedIMU-30)){
        }
        frontLeft.setPower(-0.15);
        backLeft.setPower(-0.15);
        frontRight.setPower(0.15);
        backRight.setPower(0.15);
        while(getHeading()<(wantedIMU-3)){
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);



    }



}