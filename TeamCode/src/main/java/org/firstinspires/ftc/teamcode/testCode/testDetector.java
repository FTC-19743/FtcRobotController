package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.OpenCVSignalDetector;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name = "testDetector")

public class testDetector extends LinearOpMode {
    public OpenCVSignalDetector signalDetector;



    public void runOpMode() {
        teamUtil.init(this);
        signalDetector = new OpenCVSignalDetector();
        signalDetector.initialize(true,true);
        signalDetector.activate();

        telemetry.addLine("Waiting for start");
        telemetry.update();
        long now = System.currentTimeMillis();
        while (!opModeIsActive())
        {
            signalDetector.writeTelemetry();
            sleep(100); // save cpu for OpenCV
            if (System.currentTimeMillis() > now+2000) {
                signalDetector.nextView();
                now = System.currentTimeMillis();
            }
        }
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Signal: ", signalDetector.signalDetect());
            telemetry.update();
        }
        signalDetector.deactivate();
    }
}




