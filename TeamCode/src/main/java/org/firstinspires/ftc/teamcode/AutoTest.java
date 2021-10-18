package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;



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
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor slappyArm = null;
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
    static final double     SPIN_CIRCUMFERENCE_INCHES   = 33.771;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_DEGREE       = (SPIN_CIRCUMFERENCE_INCHES / 360) *
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
        moveInches(100.0,24.0);
        spinLeft(100.0,90.0);
        spinRight(100.0,90.0);




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
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {
                log("Moving Forward");



            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
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
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {
                log("Moving Backward");



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
            newLeftTarget = leftDrive.getCurrentPosition() + (int) ((degrees * COUNTS_PER_DEGREE) * -1);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(speed*-1);
            rightDrive.setPower(speed);
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {
                log("Spinning Left");



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
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newRightTarget = rightDrive.getCurrentPosition() + (int) ((degrees * COUNTS_PER_DEGREE) * -1);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(speed);
            rightDrive.setPower(speed*-1);
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {
                log("Spinning Right");



            }


            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }

}

