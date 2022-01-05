package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp

public class MaxVelocityTest extends LinearOpMode {
    private DcMotorEx motor = null;

    double currentVelocity;

    double maxVelocity = 0.0;

    @Override

    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "duckMotor");


        // This will turn the motor at 200 ticks per second

        waitForStart();

        while (opModeIsActive()) {

            motor.setPower(100);
            currentVelocity = motor.getVelocity();

            if (currentVelocity > maxVelocity) {

                maxVelocity = currentVelocity;

            }

            telemetry.addData("current velocity", currentVelocity);

            telemetry.addData("maximum velocity", maxVelocity);

            telemetry.update();

        }
        motor.setPower(0.0);
    }

}