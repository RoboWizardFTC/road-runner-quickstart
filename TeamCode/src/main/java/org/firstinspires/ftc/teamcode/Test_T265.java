package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class Test_T265 extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }

        telemetry.addData("Init: slamra=", slamra);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {
        final int robotRadius = 9; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

        telemetry.addData("Loop: up=", up);

        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        telemetry.addData("Loop", "Field x1=(%.2f) y1=(%.2f) x2=(%.2f) y2=(%.2f)", x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);
    }

    /*
    // Sample code

    // This is the transformation between the center of the camera and the center of the robot
    Transform2d cameraToRobot = new Transform2d();
    // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
    double encoderMeasurementCovariance = 0.8;
    // Set to the starting pose of the robot
    Pose2d startingPose = new Pose2d(1, 1, new Rotation2d());

    T265Camera slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
    slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin

    // Call this when you're ready to get camera updates
    slamra.start();

    // Now we can grab our last received pose in our main thread
    while (true) {
        slamra.getLastReceivedCameraUpdate();
    }
     */


    @Override
    public void stop() {
        slamra.stop();
    }

}