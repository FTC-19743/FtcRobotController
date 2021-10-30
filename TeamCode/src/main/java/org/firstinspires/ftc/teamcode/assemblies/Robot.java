package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public TwoWheelDrive drive;
    public IntakeOutputWheel intakeOutput;
    public CarouselSpinner spinner;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Robot() {
        teamUtil.log("Constructing Robot");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new TwoWheelDrive();
        intakeOutput = new IntakeOutputWheel();
        spinner = new CarouselSpinner();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void init() {

        teamUtil.log("Initializing Robot");
        drive.initialize();
        intakeOutput.initialize();
        spinner.init();
    }

}
