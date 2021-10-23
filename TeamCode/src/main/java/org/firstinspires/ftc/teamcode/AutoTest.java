package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 */

@Autonomous(name="AutoTest")
public class AutoTest extends LinearOpMode {

    public static void log(String logString) {
        RobotLog.d("19743: " + logString);
    }
    // Declare OpMode members.

    private BNO055IMU imu; //This variable is the imu

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor slappyArm = null;
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
    static final double     SPIN_CIRCUMFERENCE_INCHES   = 33.771;
    static final double     TURN_CIRCUMFERENCE_INCHES   = 67.542;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_SPIN_DEGREE       = (SPIN_CIRCUMFERENCE_INCHES / 360) *
            COUNTS_PER_INCH;
    static final double     COUNTS_PER_TURN_DEGREE       = (TURN_CIRCUMFERENCE_INCHES / 360) *
            COUNTS_PER_INCH;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // set up our IMU
        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Your code goes here
        // Move forward code
        /*
        while(true) {
            if (runtime.seconds() > 0.5) {

                break;
            }
            else {
                leftDrive.setPower(100);
                rightDrive.setPower(100);
            }
        }
        log("Finished Moving Straight");


        runtime.reset();

        // Turn left code
        while(true) {


            if (runtime.seconds() > 0.5) {

                break;
            }
            else {
                leftDrive.setPower(0);
                rightDrive.setPower(100);
            }
        }
        log("Finished Turning Left");


        runtime.reset();

        // Turn right code
        while(true) {


            if (runtime.seconds() > 0.5) {
                runtime.reset();
                break;
            }
            else {
                leftDrive.setPower(0);
                rightDrive.setPower(100);
            }
        }

         */
        moveBackInches(1,24);
        /*
        justWait(5000);
        spinLeft(.05,90.0);

        justWait(5000);
        spinRight(.05,90.0);
        /*
        justWait(5000);
        moveBackInches(.25,24.0);
        justWait(5000);
        turnRight(.25,90.0);
        justWait(5000);
        turnLeft(.25,90.0);*/




    }
    public void justWait(int miliseconds){

        double currTime = getRuntime();
        double waitUntil = currTime + (double)(miliseconds/1000);
        while (getRuntime() < waitUntil){
        }

    }

    double getIMUHeading() {
        Orientation anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (anglesCurrent.firstAngle);
    }

    public void moveInches(double speed,
                           double inches) {
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            log("Moving Forward");
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {




            }
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
    }

    // Move Back Method
    public void moveBackInches(double speed,
                           double inches) {
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) ((inches * COUNTS_PER_INCH) * -1);
            newRightTarget = rightDrive.getCurrentPosition() + (int) ((inches * COUNTS_PER_INCH) * -1);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(speed * -1);
            rightDrive.setPower(speed * -1);
            log("Moving Backward");
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {




            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }

    // Spin Left Method
    public void spinLeft(double speed,
                           double degrees) {
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {
            newLeftTarget = leftDrive.getCurrentPosition() + (int) ((degrees * COUNTS_PER_SPIN_DEGREE) * -1);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_SPIN_DEGREE);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(speed*-1);
            rightDrive.setPower(speed);
            log("Spinning Left");
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {




            }


            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }
    // Spin Right Method
    public void spinRight(double speed,
                         double degrees) {
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_TURN_DEGREE);
            newRightTarget = rightDrive.getCurrentPosition() + (int) ((degrees * COUNTS_PER_TURN_DEGREE) * -1);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(speed);
            rightDrive.setPower(speed*-1);
            log("Spinning Right");
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {




            }


            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }

    public void turnRight(double speed,
                          double degrees) {
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_SPIN_DEGREE);
            newRightTarget = rightDrive.getCurrentPosition() + (int) ((degrees * COUNTS_PER_SPIN_DEGREE) * -1);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(speed);
            rightDrive.setPower(speed*-1);
            log("Turning Right");
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {




            }


            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }

    public void turnLeft(double speed,
                          double degrees) {
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_TURN_DEGREE);
            newRightTarget = rightDrive.getCurrentPosition() + (int) ((degrees * COUNTS_PER_TURN_DEGREE) * -1);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(speed);
            rightDrive.setPower(speed*-1);
            log("Turning Left");
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {





            }


            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }




}

