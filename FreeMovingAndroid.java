/*
 *Team 17127 Electrobotics, based in Eagan, MN. Changed 11/11/2021, at 11:33 AM.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;




@TeleOp(name="FreeMoving", group="Linear Opmode")

public class FreeMovingAndroid extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor rightDrive2 = null;
    private DcMotor DuckMotor = null;
    private Servo servo1;
    private Servo servo2;
    private DcMotorEx flipArmMotor = null;
    private DcMotor extendoArmMotor = null;
    private ColorSensor colorSensor = null;
    Orientation angles;
    Acceleration gravity;
    private BNO055IMU imu1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive1  = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightDrive1 = hardwareMap.get(DcMotor.class, "FrontRight");
        leftDrive2 = hardwareMap.get(DcMotor.class, "BackLeft");
        rightDrive2 = hardwareMap.get(DcMotor.class, "BackRight");
        DuckMotor = hardwareMap.get(DcMotor.class, "duckMotor");
        servo1 = hardwareMap.get(Servo.class, "Servo1");
        servo2 = hardwareMap.get(Servo.class, "Servo2");
        flipArmMotor  = hardwareMap.get(DcMotorEx.class, "FlipArmMotor");
        extendoArmMotor = hardwareMap.get(DcMotor.class, "ExtendoArmMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorAndDistanceSensor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        DuckMotor.setDirection(DcMotor.Direction.FORWARD);
        flipArmMotor.setDirection(DcMotorEx.Direction.FORWARD);
        extendoArmMotor.setDirection(DcMotor.Direction.FORWARD);

        leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DuckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extendoArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        BNO055IMU.Parameters imuParameters;

        imu1 = hardwareMap.get(BNO055IMU.class, "imu1");

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu1.initialize(imuParameters);
        // Prompt user to press start buton.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower=0;
            double rightPower=0;
            double duckPower= .30;
            double flipPower;
            double extendoPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double vertical = gamepad1.right_stick_y*.75;
            double horizontal  =  -gamepad1.right_stick_x*.75;
            double pivot = -gamepad1.left_stick_x*.75;
            boolean duckSpinnerRightBumper = gamepad1.right_bumper;
            boolean duckSpinnerLeftBumper = gamepad1.left_bumper;
            double slowDown = gamepad1.right_trigger;
            boolean ServoOpen = gamepad2.y;
            boolean ServoClose = gamepad2.a;
            boolean PositiveFlip = gamepad2.x;
            boolean NegativeFlip = gamepad2.b;
            double flip = -gamepad2.left_stick_y;
            double extendo  =  gamepad2.right_stick_y;
            boolean PositiveIncremental = gamepad2.right_bumper;
            boolean NegativeIncremental = gamepad2.left_bumper;
            flipPower    = (Range.clip(flip, -1.0, 1.0))*.5 ;
            if (slowDown<.10) {
                rightDrive1.setPower(-pivot + (vertical - horizontal));
                rightDrive2.setPower(-pivot + (vertical + horizontal));
                leftDrive1.setPower(pivot + (vertical + horizontal));
                leftDrive2.setPower(pivot + (vertical - horizontal));
            }
            else {
                rightDrive1.setPower((-pivot + (vertical - horizontal))*slowDown*.5);
                rightDrive2.setPower((-pivot + (vertical + horizontal))*slowDown*.5);
                leftDrive1.setPower((pivot + (vertical + horizontal))*slowDown*.5);
                leftDrive2.setPower((pivot + (vertical - horizontal))*slowDown*.5);;
            }
            extendoPower   = (Range.clip(extendo, -1.0, 1.0))*.8;
            ArmEncoder(.50, flip*50.0);
            //flipArmMotor.setPower(flipPower);
            extendoArmMotor.setPower(extendoPower);

            if (duckSpinnerRightBumper){
                DuckMotor.setPower(duckPower);
            }
            else if (duckSpinnerLeftBumper){
                DuckMotor.setPower(-duckPower);
            }
            else{
                DuckMotor.setPower(0);

            }
            if (ServoOpen) {
                servo1.setPosition(0.40);
            }
            else if (ServoClose) {
                servo1.setPosition(0);
            }
            if (PositiveFlip) {
                ArmEncoder(1.00, 50.0);
            }
            else if (NegativeFlip){
                ArmEncoder(1.00, -50.0);
            }
            if (PositiveIncremental) {
                ArmEncoder(1.00, 50.00);
            }
            else if (NegativeIncremental) {
                ArmEncoder(1.00, -50.00);
            }
            /*if (extrapower) {
                if (flipPower < 0) {
                    flipArmMotor.setPower(flipPower-.4);
                }
                if (flipPower > 0) {
                    flipArmMotor.setPower(flipPower +.4);
                }
            }
            else {
                flipArmMotor.setPower(flipPower);
            }*/
            /*else if (ServoClose) {
                servo1.setPosition(0);
            }
            if (Servo2Open) {
                servo2.setPosition(1);
            }
            else if (Servo2Close) {
                servo2.setPosition(0);
            }
            */

            // Send calculated power to wheels

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            //telehttp://192.168.43.1:8080/java/editor.html?/src/org/firstinspires/ftc/teamcode/FreeMoving.javametry.update();
            Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                    (int) (colorSensor.green() * SCALE_FACTOR),
                    (int) (colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Alpha", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            angles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu1.getGravity();
            // Display orientation info.        telemetry.addData("rot about Z", angles.firstAngle);
            telemetry.addData("rot about Z", angles.firstAngle);
            telemetry.addData("rot about Y", angles.secondAngle);
            telemetry.addData("rot about X", angles.thirdAngle);
            // Display gravitational acceleration.
            telemetry.addData("gravity (Z)", gravity.zAccel);
            telemetry.addData("gravity (Y)", gravity.yAccel);
            telemetry.addData("gravity (X)", gravity.xAccel);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
    }

    public void ArmEncoder(double ArmFlipPower, double ArmTravelLength) {
        int newArmTarget;
        if (opModeIsActive()) {
            newArmTarget = flipArmMotor.getCurrentPosition() + (int)(ArmTravelLength);
            flipArmMotor.setTargetPosition(newArmTarget);
            flipArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            telemetry.addData("Arm Velocity", "Velocity at %7f", flipArmMotor.getVelocity());

            flipArmMotor.setPower(ArmFlipPower);

            //flipArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
