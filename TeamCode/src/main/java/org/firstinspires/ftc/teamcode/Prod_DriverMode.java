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
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="Prod: Driver Mode", group="Linear Opmode")
//@Disabled
public class Prod_DriverMode extends LinearOpMode {

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

    // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-50-9-1-ratio-117-rpm-3-3-5v-encoder/
    static final double ARM_COUNTS_PER_MOTOR_REV = 1425.1;
    static final double ARM_DRIVE_GEAR_REDUCTION = 1.0 / 1.0 * 90.0 / 30.0;

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

        leftTopDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightTopDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBottomDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBottomDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = hardwareMap.get(DcMotor.class, "intake_main");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rev_intake = hardwareMap.get(DcMotor.class, "intake_rev");
        rev_intake.setDirection(DcMotor.Direction.REVERSE);
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

        double servoTickOffPosition = 0.35;
        double servoTickOnPosition = 0.0;

        double handClampOpenPosition = 1;
        double handClampClosedPosition = 0.53;

        double armExtendPower = -1.0;
        double armRetractPower = 1.0;

        int shootingWheelSpinUpTime = 3000;
        int servoTickTravelTime = 500;
        double regularShotPowerLevel = 1.0;
        double powerShotPowerLevel = 0.75;

        double intakeForwardPowerLevel = 1.0;
        double intakeReversePowerLevel = -1.0;

        double maxDrivingPower = 1 ;
        int sleepDurationArm = 250;
        int sleepDurationClaw = 500;

        int armMovementDegree = 195;
        boolean isArmExtended = false;

        servoTick.setPosition(servoTickOffPosition);
        handClampServo.setPosition(handClampOpenPosition);
        shooting.setPower(regularShotPowerLevel);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //////////////////////////////////////////////////
            // arm control
            if (gamepad1.dpad_up)
                armDrive.setPower(armExtendPower);
            else if (gamepad1.dpad_down)
                armDrive.setPower(armRetractPower);
            else
                armDrive.setPower(motorNoPowerLevel);

            //////////////////////////////////////////////////
            // hand clamp control
            if (gamepad1.dpad_left)
                handClampServo.setPosition(handClampOpenPosition);
            else if (gamepad1.dpad_right)
                handClampServo.setPosition(handClampClosedPosition);

            //////////////////////////////////////////////////
            // arm automation
            if (gamepad1.y && !isArmExtended){
                encoderMoveArm(true, armMovementDegree, 5.0);
                //sleep(sleepDurationArm);   // optional pause after each move
                handClampServo.setPosition(handClampOpenPosition);
                //sleep(sleepDurationClaw);   // optional pause after each move

                isArmExtended = true;
            }

            if (gamepad1.a && isArmExtended){
                handClampServo.setPosition(handClampClosedPosition);
                sleep(sleepDurationClaw);   // optional pause after each move
                encoderMoveArm(false, armMovementDegree, 5.0);
                //sleep(sleepDurationArm);   // optional pause after each move

                isArmExtended = false;
            }

            //////////////////////////////////////////////////
            // regular shooting control
            if (gamepad1.b) {
                //shooting.setPower(regularShotPowerLevel);
                //sleep(shootingWheelSpinUpTime);

                for (int i = 0; i < 3; i++) {
                    servoTick.setPosition(servoTickOnPosition);
                    sleep(servoTickTravelTime);
                    servoTick.setPosition(servoTickOffPosition);
                    sleep(servoTickTravelTime);
                }

                shooting.setPower(motorNoPowerLevel);
            }

            //////////////////////////////////////////////////
            // power shot control
            if  (gamepad1.x)
                shooting.setPower(powerShotPowerLevel);
            else
                shooting.setPower(regularShotPowerLevel);

            if (gamepad1.right_bumper) {
                servoTick.setPosition(servoTickOnPosition);
                sleep(servoTickTravelTime);
                servoTick.setPosition(servoTickOffPosition);
            }

            //////////////////////////////////////////////////
            // intake control
            if  (gamepad1.left_bumper) {
                intake.setPower(intakeForwardPowerLevel);
                rev_intake.setPower(intakeForwardPowerLevel);
            }
            else if (gamepad1.left_trigger > 0.5) {
                intake.setPower(intakeReversePowerLevel);
                rev_intake.setPower(intakeReversePowerLevel);
            }
            else {
                intake.setPower(motorNoPowerLevel);
                rev_intake.setPower(motorNoPowerLevel);
            }

            //////////////////////////////////////////////////
            // driving control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double leftTopPower = y + x + rx;
            double leftBottomPower = y - x + rx;
            double rightTopPower = y - x - rx;
            double rightBottomPower = y + x - rx;

            leftTopPower = leftTopPower * maxDrivingPower;
            leftBottomPower = leftBottomPower * maxDrivingPower;
            rightTopPower = rightTopPower * maxDrivingPower;
            rightBottomPower = rightBottomPower * maxDrivingPower;

            leftTopPower = Range.clip(leftTopPower, -1.0, 1.0);
            leftBottomPower = Range.clip(leftBottomPower, -1.0, 1.0);
            rightTopPower = Range.clip(rightTopPower, -1.0, 1.0);
            rightBottomPower = Range.clip(rightBottomPower, -1.0, 1.0);

            leftTopDrive.setPower(leftTopPower);
            rightTopDrive.setPower(rightTopPower);
            leftBottomDrive.setPower(leftBottomPower);
            rightBottomDrive.setPower(rightBottomPower);
        }
    }

    public void encoderMoveArm(boolean isExtend,
                               double degree,
                               double timeoutS) {
        double armPower = 0.0;
        double armSign = 1.0;
        int armTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            if (isExtend)
                armPower = armExtendPower;
            else
                armPower = armRetractPower;

            armSign = Math.signum(armPower);

            // Determine new target position, and pass to motor controller
            armTarget = armDrive.getCurrentPosition() + (int)(ARM_COUNTS_PER_MOTOR_REV * degree / 360.0 * ARM_DRIVE_GEAR_REDUCTION * armSign);
            armDrive.setTargetPosition(armTarget);

            // Turn On RUN_TO_POSITION
            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            armDrive.setPower(armPower);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    armDrive.isBusy())
            {
                // Display it for the driver.
                telemetry.addData("Target",  "Running to %7d", armTarget);
                telemetry.addData("Path",  "Running at %7d", armDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            armDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}



