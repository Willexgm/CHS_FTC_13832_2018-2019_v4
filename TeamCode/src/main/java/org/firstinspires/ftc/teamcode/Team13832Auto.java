package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="13832 Auto", group="Iterative Opmode")
public class Team13832Auto extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    BNO055IMU imu;
    Orientation angles;

    private final double THRESHOLD = 1;
    static final double     COUNTS_PER_MOTOR_REV = 210;
    static final double     WHEEL_DIAMETER_INCHES = 3.54331;
    static final double     DRIVE_GEAR_REDUCTION = 2.0;
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private double leftPower = 0;
    private double rightPower = 0;
    private double moveSpeed = 0.4;
    private double turnSpeed = 0.1;
    private double slowRatio = 0.1;
    private double slowSpeedMultiplyer = 0.2;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize the hardware variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Start tracking with gyro
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Get a reference to the angles
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Reverse one motor beacuse the motors are in diffent orienations
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Make the motors break when the power is zero
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        // Go through the path of the robot
        //move(12, moveSpeed);
        //turn(45);
        //move(12, moveSpeed);
        //turn(90);
        //move(12, moveSpeed);
        //turn(90);
        //move(12, moveSpeed);
        int target = 45;
        while(true){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //update angle


            if(target - angles.firstAngle > THRESHOLD){ // check to see if angle is less than target
                leftPower = turnSpeed;
                rightPower = -turnSpeed;
            }else{ // If the robot is in an acceptable orientation break out of loop
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                break;
            }

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Add telemety data
            telemetry.addData("right encoder", rightDrive.getCurrentPosition());
            telemetry.addData("left encoder", leftDrive.getCurrentPosition());

            telemetry.addData("Angle", angles.firstAngle);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);


    }

    @Override
    public void loop() {}

    @Override
    public void stop() {}

    // Method that when given an angle turns the robot to the correct position
    public void turn(double targetAngle) {
        // Take into account the robots current angular orientation
        double lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double a = lastAngle + targetAngle;

        // Set motors to drive without an encoder
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Turn the robot while it is not within THRESHOLD of the given angle
        while(true){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //update angle
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            if(angles.firstAngle < a - THRESHOLD){ // check to see if angle is less than target
                leftPower = -turnSpeed;
                rightPower = turnSpeed;
            }else if(angles.firstAngle >= a + THRESHOLD){ // Check to see if angle is greater than target
                leftPower = turnSpeed;
                rightPower = -turnSpeed;
            }else{ // If the robot is in an acceptable orientation break out of loop
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                break;
            }
            // Add telemety data
            telemetry.addData("right encoder", rightDrive.getCurrentPosition());
            telemetry.addData("left encoder", leftDrive.getCurrentPosition());

            telemetry.addData("Angle", angles.firstAngle);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }

        // Stop motion

    }

    public void move(double distance, double speed){
        // Determine new target position, and pass to motor controller
        int newLeftTarget = leftDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int newRightTarget = rightDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        // Set Target and Turn On RUN_TO_POSITION
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // move the robot to the target position
        while(leftDrive.isBusy() || rightDrive.isBusy()) {
            if (rightDrive.getCurrentPosition() <= newRightTarget * (1 - slowRatio) ) {
                speed = Range.clip(speed, -1.0, 1.0);
                leftDrive.setPower(speed);
                rightDrive.setPower(speed);
            } else {
                speed = Range.clip(speed, -1.0, 1.0);
                leftDrive.setPower(speed * slowSpeedMultiplyer);
                rightDrive.setPower(speed * slowSpeedMultiplyer);
            }
        }

        // Stop the robot when its done moving
        leftDrive.setPower(0);
        leftDrive.setPower(0);
    }
}
