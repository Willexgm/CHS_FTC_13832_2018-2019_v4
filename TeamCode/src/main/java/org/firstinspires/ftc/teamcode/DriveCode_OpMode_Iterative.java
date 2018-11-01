/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 */

@TeleOp(name="TeleOp DriveCode", group="Iterative Opmode")
public class DriveCode_OpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intakeDrive = null;
    private DcMotor armDrive = null;

    private int driveMode = 1; // 1 is POV, 2 is Tank
    private double maxSpeed = 1;// Increases and decreases the speed
    private double maxSpeedArm = 0.5;

    private boolean isDownD = false;
    private boolean isPressedD = false;
    private boolean isDownU = false;
    private boolean isPressedU = false;

    private boolean isDownA = false;
    private boolean isPressedA = false;
    private boolean isDownB = false;
    private boolean isPressedB = false;
    private boolean isDownY = false;
    private boolean isPressedY = false;
    private boolean intakeIsForward = false;
    private boolean intakeIsBackward = false;


    double acceleration = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake_drive");
        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double armPower;
        double intakePower;

        // Choose to drive using either Tank Mode, or POV Mode
        if(gamepad1.a){
            driveMode = 1;
            telemetry.addData("Drive Mode", "POV");
        }
        if(gamepad1.b) {
            driveMode = 2;
            telemetry.addData("Drive Mode", "Tank");
        }

        //Change the maxSpeed to speed up or slow down the motors
        if(gamepad1.dpad_down){
            isDownD = true;
        }else{
            if(isDownD){
                isPressedD = true;
            }
            isDownD = false;
        }
        if(isPressedD){
            maxSpeed = Range.clip(maxSpeed - 0.01, 0, 1);
            isPressedD = false;
        }
        if(gamepad1.dpad_up){
            isDownU = true;
        }else{
            if(isDownU){
                isPressedU = true;
            }
            isDownU = false;
        }
        if(isPressedU){
            maxSpeed = Range.clip(maxSpeed + 0.01, 0, 1);
            isPressedU = false;
        }

        //Says if the a button was pressed
        if(gamepad2.a){
            isDownA = true;
        }else{
            if(isDownA){
                isPressedA = true;
            }
            isDownA = false;
        }
        if(isPressedA){
            intakeIsForward = false;
            intakeIsBackward = true;
            isPressedA = false;
        }
        //Says if the b button was pressed
        if(gamepad2.b){
            isDownB = true;
        }else{
            if(isDownB){
                isPressedB = true;
            }
            isDownB = false;
        }
        if(isPressedB){
            intakeIsForward = false;
            intakeIsBackward = false;
            isPressedB = false;
        }
        //Says if the c button was pressed
        if(gamepad2.y){
            isDownY = true;
        }else{
            if(isDownY){
                isPressedY = true;
            }
            isDownY = false;
        }
        if(isPressedY){
            intakeIsForward = true;
            intakeIsBackward = false;
            isPressedY = false;
        }

        if(intakeIsBackward){
            intakePower = -1;
        }else if(intakeIsForward){
            intakePower = 1;
        }else{
            intakePower = 0;
        }

        //Change the acceleration of the motors so the robot does not jerk
        if(gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0){
            acceleration += 0.01;
        }else if(acceleration > 0){
            acceleration -= 0.01;
        }

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        if(driveMode == 1) {
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            leftPower = (Range.clip((drive + turn) * acceleration, -1.0, 1.0)) * maxSpeed;
            rightPower = (Range.clip((drive - turn) * acceleration, -1.0, 1.0)) * maxSpeed;
        }else{ //ensures that there is always a value for left and right power
            leftPower  = (-gamepad1.left_stick_y) * acceleration * maxSpeed ;
            rightPower = (-gamepad1.right_stick_y) * acceleration * maxSpeed ;
        }

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        if(driveMode == 2) {
            leftPower  = Range.clip((-gamepad1.left_stick_y) * acceleration * maxSpeed, -1.0, 1.0);
            rightPower = Range.clip((-gamepad1.right_stick_y) * acceleration * maxSpeed, -1.0, 1.0);
        }

        armPower = Range.clip((-gamepad2.right_stick_y) * maxSpeedArm, -1.0, 1.0);
        //intakePower = Range.clip((-gamepad2.left_stick_y), -1.0, 1.0);

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        intakeDrive.setPower(intakePower);
        armDrive.setPower(armPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f), arm(%.2f), intake(%.2f)", leftPower, rightPower, armPower, intakePower);
    }

    @Override
    public void stop() {
    }

}
