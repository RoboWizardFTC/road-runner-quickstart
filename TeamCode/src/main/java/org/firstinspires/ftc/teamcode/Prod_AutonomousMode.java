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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.Servo;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.Math;


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

@TeleOp(name="Prod: Autonomous Mode", group="Linear Opmode")
//@Disabled
public class Prod_AutonomousMode extends LinearOpMode {

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

    OpenCvWebcam webCam;
    SkystoneDeterminationPipeline pipeline;

    // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-435-rpm-3-3-5v-encoder/
    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // 96mm
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1416);

    // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-50-9-1-ratio-117-rpm-3-3-5v-encoder/
    static final double ARM_COUNTS_PER_MOTOR_REV = 1425.1;
    static final double ARM_DRIVE_GEAR_REDUCTION = 1.0 / 1.0 * 90.0 / 30.0;

    static final double verticalDrivingMultiplier = 1;
    static final double horizontalDrivingMultiplier = Math.sqrt(2.0);

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

        double servoTickOffPosition = 0.35;
        double servoTickOnPosition = 0.0;

        double handClampOpenPosition = 1;
        double handClampClosedPosition = 0.53;

        int shootingWheelSpinUpTime = 3250;
        int servoTickTravelTime = 750;
        double regularShotPowerLevel = 1.0;
        double powerShotPowerLevel = 0.8;

        double intakeForwardPowerLevel = 1.0;
        double intakeReversePowerLevel = -1.0;

        double maxDrivingPowerBeforeShooting = 0.5;
        double maxDrivingPower = 0.7;
        double maxDrivingPowerLeftRight = 0.5;

        int sleepDurationMovement = 250;
        int sleepDurationArm = 250;
        int sleepDurationClaw = 500;

        int armMovementDegree = 180;

        servoTick.setPosition(servoTickOffPosition);
        handClampServo.setPosition(handClampClosedPosition);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webCam.setPipeline(pipeline);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        leftTopDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBottomDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTopDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBottomDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

            SkystoneDeterminationPipeline.RingPosition ringPosition = SkystoneDeterminationPipeline.RingPosition.NONE;

            while (runtimeDetectRingStack.seconds() < 3.0) {
                ringPosition = pipeline.position;

                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.addData("Position", ringPosition);
                telemetry.update();

                // Don't burn CPU cycles busy-looping in this sample
                sleep(50);
            }

            // move forward to shooting position
            encoderDrive(DrivingDirection.FORWARD, maxDrivingPowerBeforeShooting,57, 5.0);
            sleep(sleepDurationMovement);   // optional pause after each move
            encoderDrive(DrivingDirection.RIGHT, maxDrivingPowerLeftRight,5, 5.0);
            sleep(sleepDurationMovement);   // optional pause after each move

            // shoot three rings
            shooting.setPower(regularShotPowerLevel);
            sleep(shootingWheelSpinUpTime);

            for (int i = 0; i < 3; i++) {
                servoTick.setPosition(servoTickOnPosition);
                sleep(servoTickTravelTime);
                servoTick.setPosition(servoTickOffPosition);
                sleep(servoTickTravelTime);
            }

            shooting.setPower(motorNoPowerLevel);

            // move to the right square

            switch (ringPosition) {
                case FOUR:
                    encoderDrive(DrivingDirection.FORWARD, maxDrivingPower,39, 5.0);
                    sleep(sleepDurationMovement);   // optional pause after each move
                    encoderDrive(DrivingDirection.LEFT, maxDrivingPowerLeftRight,21, 5.0);
                    sleep(sleepDurationMovement);   // optional pause after each move
                    break;
                case ONE:
                    encoderDrive(DrivingDirection.FORWARD, maxDrivingPower,19, 5.0);
                    sleep(sleepDurationMovement);   // optional pause after each move
                    break;
                case NONE:
                    encoderDrive(DrivingDirection.BACKWARD, maxDrivingPower,4, 5.0);
                    sleep(sleepDurationMovement);   // optional pause after each move
                    encoderDrive(DrivingDirection.LEFT, maxDrivingPowerLeftRight,21, 5.0);
                    sleep(sleepDurationMovement);   // optional pause after each move
                    break;

            }

            // place the wobble goal
            encoderMoveArm(true, armMovementDegree, 5.0);
            sleep(sleepDurationArm);   // optional pause after each move
            handClampServo.setPosition(handClampOpenPosition);
            sleep(sleepDurationClaw);   // optional pause after each move

            // park robot on the launch line
            switch (ringPosition) {
                case FOUR:
                    encoderDrive(DrivingDirection.RIGHT, maxDrivingPowerLeftRight,19, 5.0);
                    sleep(sleepDurationMovement);
                    encoderDrive(DrivingDirection.BACKWARD, maxDrivingPower,27, 5.0);
                    sleep(sleepDurationMovement);   // optional pause after each move
                    break;
                case ONE:
                    encoderDrive(DrivingDirection.BACKWARD, maxDrivingPower,8, 5.0);
                    sleep(sleepDurationMovement);   // optional pause after each move
                    break;
                case NONE:
                    encoderDrive(DrivingDirection.RIGHT, maxDrivingPowerLeftRight,19, 5.0);
                    sleep(sleepDurationMovement);   // optional pause after each move
                    encoderDrive(DrivingDirection.FORWARD, maxDrivingPower,15, 5.0);
                    sleep(sleepDurationMovement);
                    break;

            }

            encoderMoveArm(false, armMovementDegree, 5.0);
            sleep(sleepDurationArm);   // optional pause after each move

            break;
        }

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(DrivingDirection direction,
                             double power,
                             double distance,
                             double timeoutS) {
        double leftTopPower = 0.0;
        double leftBottomPower = 0.0;
        double rightTopPower = 0.0;
        double rightBottomPower = 0.0;

        double leftTopSign = 1.0;
        double leftBottomSign = 1.0;
        double rightTopSign = 1.0;
        double rightBottomSign = 1.0;

        double encoderMultiplier = 1.0;

        int leftTopTarget = 0;
        int leftBottomTarget = 0;
        int rightTopTarget = 0;
        int rightBottomTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            switch (direction) {
                case FORWARD:
                    leftTopPower = power;
                    leftBottomPower = power;
                    rightTopPower = power;
                    rightBottomPower = power;
                    encoderMultiplier = verticalDrivingMultiplier;
                    break;
                case BACKWARD:
                    leftTopPower = -power;
                    leftBottomPower = -power;
                    rightTopPower = -power;
                    rightBottomPower = -power;
                    encoderMultiplier = verticalDrivingMultiplier;
                    break;
                case LEFT:
                    leftTopPower = -power;
                    leftBottomPower = power;
                    rightTopPower = power;
                    rightBottomPower = -power;
                    encoderMultiplier = horizontalDrivingMultiplier;
                    break;
                case RIGHT:
                    leftTopPower = power;
                    leftBottomPower = -power;
                    rightTopPower = -power;
                    rightBottomPower = power;
                    encoderMultiplier = horizontalDrivingMultiplier;
                    break;
            }

            leftTopSign = Math.signum(leftTopPower);
            leftBottomSign = Math.signum(leftBottomPower);
            rightTopSign = Math.signum(rightTopPower);
            rightBottomSign = Math.signum(rightBottomPower);

            // Determine new target position, and pass to motor controller
            leftTopTarget = leftTopDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH * encoderMultiplier * leftTopSign);
            leftBottomTarget = leftBottomDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH * encoderMultiplier * leftBottomSign);
            rightTopTarget = rightTopDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH * encoderMultiplier * rightTopSign);
            rightBottomTarget = rightBottomDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH * encoderMultiplier * rightBottomSign);

            leftTopDrive.setTargetPosition(leftTopTarget);
            leftBottomDrive.setTargetPosition(leftBottomTarget);
            rightTopDrive.setTargetPosition(rightTopTarget);
            rightBottomDrive.setTargetPosition(rightBottomTarget);

            // Turn On RUN_TO_POSITION
            leftTopDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBottomDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightTopDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBottomDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftTopDrive.setPower(leftTopPower);
            leftBottomDrive.setPower(leftBottomPower);
            rightTopDrive.setPower(rightTopPower);
            rightBottomDrive.setPower(rightBottomPower);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    leftTopDrive.isBusy() &&
                    leftBottomDrive.isBusy() &&
                    rightTopDrive.isBusy() &&
                    rightBottomDrive.isBusy())
            {
                // Display it for the driver.
                telemetry.addData("Target",  "Running to %7d :%7d :%7d :%7d",
                        leftTopTarget,  leftBottomTarget, rightTopTarget, rightBottomTarget);
                telemetry.addData("Path",  "Running at %7d :%7d :%7d :%7d",
                        leftTopDrive.getCurrentPosition(),
                        leftBottomDrive.getCurrentPosition(),
                        rightTopDrive.getCurrentPosition(),
                        rightBottomDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftTopDrive.setPower(0);
            leftBottomDrive.setPower(0);
            rightTopDrive.setPower(0);
            rightBottomDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftTopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightTopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        private Gamepad gamepad1;

        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */

        //dont touch. it works

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(90,135);

        static final int REGION_WIDTH = 63;
        static final int REGION_HEIGHT = 45;

        final int FOUR_RING_THRESHOLD = 144;
        final int ONE_RING_THRESHOLD = 126;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SkystoneDeterminationPipeline.RingPosition position = SkystoneDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            int returnValue;

            position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                returnValue = 4;
                position = SkystoneDeterminationPipeline.RingPosition.FOUR;

            }else if (avg1 > ONE_RING_THRESHOLD){
                position = SkystoneDeterminationPipeline.RingPosition.ONE;
                returnValue = 1;
            }else{
                position = SkystoneDeterminationPipeline.RingPosition.NONE;
                returnValue = 0;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
            //input return is a void there
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

}



