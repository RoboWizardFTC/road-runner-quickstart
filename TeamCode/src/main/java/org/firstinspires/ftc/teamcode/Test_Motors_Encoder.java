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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test: Motors Encoder", group="Linear Opmode")
//@Disabled
public class Test_Motors_Encoder extends LinearOpMode {

    public enum DrivingDirection
    {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftTopDrive = null;
    private DcMotor leftBottomDrive = null;
    private DcMotor rightBottomDrive = null;
    private DcMotor rightTopDrive = null;

    private DcMotor intake = null;
    private DcMotor rev_intake = null;
    private DcMotor shooting = null;

    private DcMotor armDrive = null;
    private Servo handClampServo = null;

    private Servo servoTick = null;

    // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-312-rpm-3-3-5v-encoder/
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // 96mm
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1416);

    // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-223-rpm-3-3-5v-encoder/
    static final double ARM_COUNTS_PER_MOTOR_REV = 751.8;
    static final double ARM_DRIVE_GEAR_REDUCTION = 2.0 / 1.0 * 90.0 / 30.0;

    static final double verticalDrivingMultiplier = 1.0;
    static final double horizontalDrivingMultiplier = 1.0; //Math.sqrt(2.0);

    static final double armExtendPower = -1.0;
    static final double armRetractPower = 1.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftTopDrive = hardwareMap.get(DcMotor.class, "left_top_drive");
        leftBottomDrive = hardwareMap.get(DcMotor.class, "left_bottom_drive");
        rightTopDrive = hardwareMap.get(DcMotor.class, "right_top_drive");
        rightBottomDrive = hardwareMap.get(DcMotor.class, "right_bottom_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftTopDrive.setDirection(DcMotor.Direction.REVERSE);
        rightTopDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBottomDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBottomDrive.setDirection(DcMotor.Direction.FORWARD);

        leftTopDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTopDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBottomDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBottomDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake_main");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rev_intake = hardwareMap.get(DcMotor.class, "intake_rev");
        rev_intake.setDirection(DcMotor.Direction.FORWARD);
        rev_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooting = hardwareMap.get(DcMotor.class, "shooting");
        shooting.setDirection(DcMotor.Direction.REVERSE);
        shooting.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoTick = hardwareMap.get(Servo.class, "servo_tick");

        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");
        armDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        handClampServo = hardwareMap.get(Servo.class, "hand_clamp");

        double motorNoPowerLevel = 0.0;

        double servoTickOffPosition = 0.0;
        double servoTickOnPosition = 0.30;

        double handClampOpenPosition = 0.22;
        double handClampClosedPosition = 0.63;

        int shootingWheelSpinUpTime = 3000;
        int servoTickTravelTime = 500;
        double regularShotPowerLevel = 1.0;
        double powerShotPowerLevel = 0.8;

        double intakeForwardPowerLevel = 1.0;
        double intakeReversePowerLevel = -1.0;

        double maxDrivingPowerBeforeShooting = 0.4;
        double maxDrivingPower = 0.4;
        int sleepDurationMovement = 250;
        int sleepDurationArm = 250;
        int sleepDurationClaw = 500;

        servoTick.setPosition(servoTickOffPosition);
        handClampServo.setPosition(handClampClosedPosition);

        leftTopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {
            // take up to 3 seconds to detect the ring stack
            ElapsedTime runtimeDetectRingStack = new ElapsedTime();
            runtimeDetectRingStack.reset();

            if (gamepad1.x)
                leftTopDrive.setPower(1);
            else
                leftTopDrive.setPower(0);

            if (gamepad1.y)
                leftBottomDrive.setPower(1);
            else
                leftBottomDrive.setPower(0);

            if (gamepad1.a)
                rightTopDrive.setPower(1);
            else
                rightTopDrive.setPower(0);

            if (gamepad1.b)
                rightBottomDrive.setPower(1);
            else
                rightBottomDrive.setPower(0);
        }

    }

}



