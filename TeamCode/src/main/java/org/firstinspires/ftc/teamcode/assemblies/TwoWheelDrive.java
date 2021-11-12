package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public class TwoWheelDrive {
    HardwareMap hardwareMap;

    private BNO055IMU imu; //This variable is the imu
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    static final double COUNTS_PER_MOTOR_REV = 384.5;    // GoBilda 5202 435 RPM
    static final double SPIN_CIRCUMFERENCE_INCHES = 33.771;
    static final double TURN_CIRCUMFERENCE_INCHES = 67.542;
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // 2:1 gear train reduction
    static final double WHEEL_DIAMETER_INCHES = 6.0;     // Large AndyMark Wheels
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_SPIN_DEGREE = (SPIN_CIRCUMFERENCE_INCHES / 360) *
            COUNTS_PER_INCH;
    static final double COUNTS_PER_TURN_DEGREE = (TURN_CIRCUMFERENCE_INCHES / 360) *
            COUNTS_PER_INCH;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public TwoWheelDrive() {
        teamUtil.log("Constructing Drive");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initialize() {

        teamUtil.log("Initializing Drive");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // set up our IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        teamUtil.log("Initializing Drive - FINISHED");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double getIMUHeading() {
        Orientation anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return (anglesCurrent.firstAngle);
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void manualControl(float power, float spin) {
        double leftPower;
        double rightPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = power;
        double turn = spin;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveInches(double speed,
                           double inches) {

        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newRightTarget = rightDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));
        teamUtil.log("Moving Forward");
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (leftDrive.isBusy() || rightDrive.isBusy())) {
            teamUtil.log("waiting");
        }
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Move Back Method
    public void moveBackInches(double speed,
                               double inches) {
        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() + (int) ((inches * COUNTS_PER_INCH) * -1);
        newRightTarget = rightDrive.getCurrentPosition() + (int) ((inches * COUNTS_PER_INCH) * -1);
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        leftDrive.setPower(speed * -1);
        rightDrive.setPower(speed * -1);
        teamUtil.log("Moving Backward");
        while (teamUtil.keepGoing(5000) && (leftDrive.isBusy() && rightDrive.isBusy())) {
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Spin Left Method
    public void spinLeft(double speed,
                         double degrees) {
        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = leftDrive.getCurrentPosition() + (int) ((degrees * COUNTS_PER_SPIN_DEGREE) * -1);
        newRightTarget = rightDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_SPIN_DEGREE);
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        leftDrive.setPower(speed * -1);
        rightDrive.setPower(speed);
        teamUtil.log("Spinning Left");
        while (teamUtil.keepGoing(5000) && (leftDrive.isBusy() && rightDrive.isBusy())) {
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Spin Right Method
    public void spinRight(double speed,
                          double degrees) {
        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = leftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_TURN_DEGREE);
        newRightTarget = rightDrive.getCurrentPosition() + (int) ((degrees * COUNTS_PER_TURN_DEGREE) * -1);
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        leftDrive.setPower(speed);
        rightDrive.setPower(speed * -1);
        teamUtil.log("Spinning Right");
        while (teamUtil.keepGoing(5000) && (leftDrive.isBusy() && rightDrive.isBusy())) {
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnRight(double speed,
                          double degrees) {
        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = leftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_SPIN_DEGREE);
        newRightTarget = rightDrive.getCurrentPosition() + (int) ((degrees * COUNTS_PER_SPIN_DEGREE) * -1);
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        leftDrive.setPower(speed);
        rightDrive.setPower(speed * -1);
        teamUtil.log("Turning Right");
        while (teamUtil.keepGoing(5000) && (leftDrive.isBusy() && rightDrive.isBusy())) {
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnLeft(double speed,
                         double degrees) {
        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = leftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_TURN_DEGREE);
        newRightTarget = rightDrive.getCurrentPosition() + (int) ((degrees * COUNTS_PER_TURN_DEGREE) * -1);
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        leftDrive.setPower(speed);
        rightDrive.setPower(speed * -1);
        teamUtil.log("Turning Left");
        while (teamUtil.keepGoing(5000) && (leftDrive.isBusy() && rightDrive.isBusy())) {
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void spinRightWithIMU(double degrees, double speed) {

        double currentIMU = getIMUHeading();
        double targetIMU;
        if (currentIMU - degrees >= -180)
        {
            targetIMU = degrees - currentIMU + 360;
        }
        else{
            targetIMU = degrees - currentIMU;
        }
        double distance = targetIMU - currentIMU;
        if (distance >= 0){
            return;
        }
        else{
            distance = -1*(distance);
        }
        while (distance <= .5) {
            distance = targetIMU - currentIMU;
            if (distance >= 0){
                return;
            }
            else{
                distance = -1*(distance);
            }
            leftDrive.setPower(speed);
            rightDrive.setPower(speed*-1);
            currentIMU = getIMUHeading();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }


    /*
    public void spinLeftWithIMU(double degrees, double speed) {

    }

        getIMUHeading(true);
        double currentIMU = getIMUHeading(false);
        if(degrees>=180) {
            while(currentIMU<180) {
                currentIMU = getIMUHeading(false);
                leftDrive.setPower(speed);
                rightDrive.setPower(speed*-1);
            }
            double degreesLeft = degrees - currentIMU;
            double finalHeading = -180 - degreesLeft;
            while(currentIMU>finalHeading) {
                currentIMU = getIMUHeading(false);
                leftDrive.setPower(speed);
                rightDrive.setPower(speed*-1);
            }

        }
        else{
            while(currentIMU<degrees) {
                currentIMU = getIMUHeading(false);
                leftDrive.setPower(speed);
                rightDrive.setPower(speed*-1);
            }
        }
    }
    */


    public void spinLeftWithIMU(double degrees, double speed) {
        double currentIMU = getIMUHeading();
        double targetIMU;
        if (currentIMU + degrees >= 180)
        {
            targetIMU = degrees + currentIMU - 360;
        }
        else{
            targetIMU = degrees + currentIMU;
        }
        double distance = targetIMU - currentIMU;
        if (distance >= 0){

        }
        else{
            distance = -1*(distance);
        }

        while (distance >= .5) {
            distance = targetIMU - currentIMU;
            String distanceLog = String.format("%.2f", distance);
            teamUtil.log(distanceLog);
            if (distance >= 0){
            }
            else{
                distance = -1*(distance);
            }
            leftDrive.setPower(speed*-1);
            rightDrive.setPower(speed);
            currentIMU = getIMUHeading();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void spinRightWithIMUV2(double degrees, double speed){
        double currentIMU = getIMUHeading();
        double initialIMU = getIMUHeading();
        //code for when imu is positive and right spin is needed that will put ultimately
        //put imu into positive once again
        if(currentIMU>=0 && currentIMU-degrees<180){
            while(currentIMU>-180){
                currentIMU=getIMUHeading();
                leftDrive.setPower(speed);
                rightDrive.setPower(-1*speed);
                //prints IMU after every instance of while loop
                String IMUToPrint1 = String.format("%.2f", getIMUHeading());
                log(IMUToPrint1);

            double degreesTurned = initialIMU*-1-180;
            double degreesStillNeeded = degrees-degreesTurned;
            double IMUNeeded = 179.999999-degreesStillNeeded;
            while(currentIMU>IMUNeeded){
                currentIMU=getIMUHeading();
                leftDrive.setPower(speed);
                rightDrive.setPower(-1*speed);
                //prints IMU after every instance of while loop
                String IMUToPrint2 = String.format("%.2f", getIMUHeading());
                log(IMUToPrint2);
            }


            }
        }
        else if(currentIMU>=0 && currentIMU-degrees>=180){
            while(currentIMU>degrees) {
                currentIMU = getIMUHeading();
                leftDrive.setPower(speed);
                rightDrive.setPower(-1 * speed);
                //prints IMU after every instance of while loop
                String IMUToPrint1 = String.format("%.2f", getIMUHeading());
                log(IMUToPrint1);
            }
        }




    }


}
