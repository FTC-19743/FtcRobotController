package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name="MecanumTeleopTest", group="Linear Opmode")

public class MecanumTeleopTest extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    public void runOpMode() {
        //teamUtil.init(this);

        //These are the parameters that the imu uses in the code to name and keep track of the data
        frontLeftMotor = hardwareMap.get(DcMotor.class, "flm");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frm");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blm");
        backRightMotor = hardwareMap.get(DcMotor.class, "brm");

        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);


        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double y = -gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.left_stick_y; // Remember, this is reversed!

            if(gamepad1.right_bumper == true){
                frontLeftMotor.setPower(y + x + rx);
                backLeftMotor.setPower(y - x + rx);
                frontRightMotor.setPower(y - x - rx);
                backRightMotor.setPower(y + x - rx);
            } else{
                frontLeftMotor.setPower((y + x + rx)/2);
                backLeftMotor.setPower((y - x + rx)/2);
                frontRightMotor.setPower((y - x - rx)/2);
                backRightMotor.setPower((y + x - rx)/2);
            }

        }
    }

}
