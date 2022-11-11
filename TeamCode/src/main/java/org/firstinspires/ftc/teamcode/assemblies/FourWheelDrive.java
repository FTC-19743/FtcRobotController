package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

    public BNO055IMU imu; //This variable is the imu
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

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


    }
    /*
    public void updatePowers(){
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }

     */

    public void strafeLeft(double speed, double centimeters){
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
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy()||backLeft.isBusy()||backRight.isBusy())) {

            teamUtil.log("waiting");
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

    public void strafeRight(double speed, double centimeters){
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
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy()||backLeft.isBusy()||backRight.isBusy())) {

            teamUtil.log("waiting");
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

    public void outputTelemetry(){
        telemetry.addData("Output  ", "flm:%d frm:%d blm:%d brm:%d",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),backLeft.getCurrentPosition(), backRight.getCurrentPosition());
    }

    //basic move centimeters without accel and deceleration
    public void moveCM(double speed, double centimeters){
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
        while (teamUtil.keepGoing(currentTime) && (frontLeft.isBusy() || frontRight.isBusy()||backLeft.isBusy()||backRight.isBusy())) {

            teamUtil.log("waiting");
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

    public void backCM(double speed, double centimeters){
        moveCM(speed, -centimeters);
    }

    public void runMotors(double velocity){
        frontLeft.setVelocity(velocity);
        frontRight.setVelocity(velocity);
        backLeft.setVelocity(velocity);
        backRight.setVelocity(velocity);
    }



    public void moveCmWAcceleration(double cruiseVelocity, double centimeters){
        double startEncoderPosition = frontLeft.getCurrentPosition();



        double velocityChangeNeededAccel = cruiseVelocity-MIN_START_VELOCITY;
        double velocityChangeNeededDecel = cruiseVelocity-MIN_END_VELOCITY;


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double totalTics = centimeters*COUNTS_PER_CENTIMETER;
        double ticsDuringAcceleration = velocityChangeNeededAccel/MAX_ACCELERATION;
        double ticsDuringDeceleration = velocityChangeNeededDecel/MAX_DECELERATION;
        double cruiseTics = totalTics-ticsDuringDeceleration-ticsDuringAcceleration;

        while(frontLeft.getCurrentPosition()<startEncoderPosition+ticsDuringAcceleration){
            double ticsSinceStart = frontLeft.getCurrentPosition()-startEncoderPosition;

            runMotors(MAX_ACCELERATION*ticsSinceStart+MIN_START_VELOCITY);
        }

        while(frontLeft.getCurrentPosition()<cruiseTics+startEncoderPosition){
            runMotors(cruiseVelocity);
        }

        double encoderAfterCruise = frontLeft.getCurrentPosition();

        while(frontLeft.getCurrentPosition()<startEncoderPosition+totalTics){
            double ticsSinceCruise = frontLeft.getCurrentPosition()-encoderAfterCruise;

            runMotors(MAX_DECELERATION*ticsSinceCruise+MIN_END_VELOCITY);
        }
        runMotors(0);

    }


}