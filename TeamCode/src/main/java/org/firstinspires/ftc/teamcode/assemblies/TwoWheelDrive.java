package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
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
public class TwoWheelDrive {
    HardwareMap hardwareMap;
    Telemetry telemetry;

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

    public void writeTelemetry(){
        telemetry.addData("Left Motor Position", "Wheel Position:%d", leftDrive.getCurrentPosition());
        telemetry.addData("Right Motor Position", "Wheel Position:%d", rightDrive.getCurrentPosition());
        //telemetry.addData("Left Motor Speed", "Wheel Speed:%f", leftDrive.getPower());
        //telemetry.addData("Right Motor Speed", "Wheel Speed:%f", rightDrive.getPower());
        telemetry.addData("IMU Heading", "IMU Value:%f", getIMUHeading());


    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public static void detailLog(String logString){
        RobotLog.d("DetailLOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    public TwoWheelDrive() {
        teamUtil.log("Constructing Drive");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
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
        //teamUtil.log("manualControl - P:"+power+" S:"+ spin);

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = power;
        double turn = spin;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        //teamUtil.log("manualControl - LP:"+leftPower+" RP:"+ rightPower);

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
            String leftCurrentPosition = String.format("%d", leftDrive.getCurrentPosition());
            String rightCurrentPosition = String.format("%d", rightDrive.getCurrentPosition());
            detailLog("Left Position: " + leftCurrentPosition);
            detailLog("Right Position: " + rightCurrentPosition);
            teamUtil.log("waiting");
        }

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setPower(0);
        rightDrive.setPower(0);


    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Move Back Method
    public void moveBackInches(double speed,
                               double inches) {
        int newLeftTarget;
        int newRightTarget;
        String leftCurrentPosition = String.format("%d", leftDrive.getCurrentPosition());
        String rightCurrentPosition = String.format("%d", rightDrive.getCurrentPosition());

        detailLog("Left Position: " + leftCurrentPosition);
        detailLog("Right Position: " + rightCurrentPosition);


        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
        newRightTarget = rightDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
        String leftTarget = String.format("%d",newLeftTarget);
        String rightTarget = String.format("%d",newRightTarget);
        detailLog("Left Target: " + leftTarget);
        detailLog("Right Target: " + rightTarget);
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        leftDrive.setPower(speed * -1);
        rightDrive.setPower(speed * -1);
        teamUtil.log("Moving Backward");
        long currentTime = System.currentTimeMillis() + 5000;
        while (teamUtil.keepGoing(currentTime) && (leftDrive.isBusy() || rightDrive.isBusy())) {

            detailLog("Left Position: " + leftCurrentPosition);
            detailLog("Right Position: " + rightCurrentPosition);
            teamUtil.log("waiting");
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
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double initialIMU = getIMUHeading();
        double IMUNeeded = initialIMU-degrees;
        String initialIMUToPrint = String.format("%.2f", initialIMU);
        String IMUNeededToPrint = String.format("%.2f", IMUNeeded);
        detailLog("Initial IMU: " + initialIMUToPrint);
        detailLog("IMU Needed: " + IMUNeededToPrint);
        //if the end IMU is an impossible value, this code allows the robot to transition
        //from the left hemisphere to the right without issues
        //IMU Diagram Doc: https://docs.google.com/document/d/1RI6dZkmHRWhUBy-ZgONwAEO7AxOb_vjcoX40VSjJYjg/edit
        if(IMUNeeded<-180){
            double currentIMU = getIMUHeading();
            while(currentIMU!=179.999999){
                currentIMU=getIMUHeading();
                leftDrive.setPower(speed);
                rightDrive.setPower(-1*speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
                detailLog(currentIMUToPrint);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            //finds out how many degrees were traveled depending on where the robot was initially
            //facing
            double degreesTraveled=0;
            if(initialIMU < 0){
                degreesTraveled = 180+initialIMU;
            }
            else if(initialIMU==0){

            }
            else{
                degreesTraveled = 180-initialIMU;
            }
            currentIMU=getIMUHeading();
            //Finds out how far to travel and prints important values
            double degreesLeft = degrees-degreesTraveled;
            double IMUNeeded2 = 179.999999-degreesLeft;
            String degreesTraveledToPrint = String.format("%.2f", degreesTraveled);
            String degreesLeftToPrint = String.format("%.2f", degreesLeft);
            String IMUNeeded2ToPrint = String.format("%.2f", IMUNeeded2);
            detailLog("Degrees Traveled: " + degreesTraveledToPrint);
            detailLog("Degrees Left To Travel " + degreesLeftToPrint);
            detailLog("IMU needed as robot enters left hemisphere: " + IMUNeeded2ToPrint);

            while(currentIMU>IMUNeeded2){
                currentIMU=getIMUHeading();
                leftDrive.setPower(speed);
                rightDrive.setPower(-1*speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
                detailLog(currentIMUToPrint);

            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

        }
        //If math is simple and no conversion is needed the robot will spin without issue
        else{
            double currentIMU = getIMUHeading();
            while(currentIMU>IMUNeeded){
                currentIMU=getIMUHeading();
                leftDrive.setPower(speed);
                rightDrive.setPower(-1*speed);
                String currentIMUToPrint = String.format("%.2f", currentIMU);
                detailLog(currentIMUToPrint);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }


        /*
        while (true) {

            if(degrees!=0){
                while(currentIMU!=360){

                    currentIMU=getIMUHeading();
                    String IMUToPrint = String.format("%.2f", getIMUHeading());
                    log(IMUToPrint);
                    String InitialIMUToPrint = String.format("%.2f",currentIMU);
                    log(InitialIMUToPrint);
                    rightDrive.setPower(-speed);
                    leftDrive.setPower(speed);
                }
            }
        }

        //code for when imu is positive and right spin is needed that will put ultimately
        //put imu into positive once again
        if(currentIMU>=0 && currentIMU-degrees<180){
            while(currentIMU<0) {
                currentIMU = getIMUHeading();
                leftDrive.setPower(speed);
                rightDrive.setPower(-1 * speed);
                //prints IMU after every instance of while loop
                String IMUToPrint1 = String.format("%.2f", currentIMU);
                log(IMUToPrint1);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            double degreesTurned = initialIMU*-1-180;
            double degreesStillNeeded = degrees-degreesTurned;
            double IMUNeeded = 179.999999-degreesStillNeeded;
            String degreesTurnedToPrint = String.format("%.2f", degreesTurned);
            log(degreesTurnedToPrint);
            String degreesStillNeededToPrint = String.format("%.2f", degreesStillNeeded);
            log(degreesStillNeededToPrint);
            String IMUNeededToPrint = String.format("%.2f", IMUNeeded);
            log(IMUNeededToPrint);
            while(currentIMU>IMUNeeded){
                getIMUHeading();
                currentIMU=getIMUHeading();
                leftDrive.setPower(speed);
                rightDrive.setPower(-1*speed);
                //prints IMU after every instance of while loop
                String IMUToPrint2 = String.format("%.2f", currentIMU);
                log(IMUToPrint2);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);


        }
        else if(currentIMU>=0 && currentIMU-degrees>=180){
            while(currentIMU>degrees) {
                currentIMU = getIMUHeading();
                leftDrive.setPower(speed);
                rightDrive.setPower(-1*speed);
                //prints IMU after every instance of while loop
                String IMUToPrint1 = String.format("%.2f", currentIMU);
                log(IMUToPrint1);
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        */
        




    }

    public void spinLeftWithIMUV2(double degrees, double speed){
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double initialIMU = getIMUHeading();
        double IMUNeeded = initialIMU+degrees;
        String initialIMUToPrint = String.format("%.2f", initialIMU);
        String IMUNeededToPrint = String.format("%.2f", IMUNeeded);
        detailLog("Initial IMU: " + initialIMUToPrint);
        detailLog("IMU Needed: " + IMUNeededToPrint);

        //IMU Diagram Doc: https://docs.google.com/document/d/1RI6dZkmHRWhUBy-ZgONwAEO7AxOb_vjcoX40VSjJYjg/edit
        if(IMUNeeded>180){
            double currentIMU = getIMUHeading();
            while(currentIMU!=179.999){
                currentIMU=getIMUHeading();
                leftDrive.setPower(-1*speed);
                rightDrive.setPower(speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
                detailLog(currentIMUToPrint);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
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
            detailLog("Degrees Traveled: " + degreesTraveledToPrint);
            detailLog("Degrees Left To Travel " + degreesLeftToPrint);
            detailLog("IMU needed as robot enters left hemisphere: " + IMUNeeded2ToPrint);

            while(currentIMU<IMUNeeded2){
                currentIMU=getIMUHeading();
                leftDrive.setPower(-1*speed);
                rightDrive.setPower(speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
                detailLog(currentIMUToPrint);

            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

        }
        //If math is simple and no conversion is needed the robot will spin without issue
        else{
            double currentIMU = getIMUHeading();
            while(currentIMU<IMUNeeded){
                currentIMU=getIMUHeading();
                leftDrive.setPower(-1*speed);
                rightDrive.setPower(speed);
                String currentIMUToPrint = String.format("%.2f", initialIMU);
                detailLog(currentIMUToPrint);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }








    }
}
