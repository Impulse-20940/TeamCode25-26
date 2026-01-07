package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Camera {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;
    public static final boolean USE_WEBCAM = true;
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    AprilTagProcessor aprilTag;

    VisionPortal visionPortal;
    double[] shift = {-40, -380, -2};
    private void initAprilTag() {
        double fx = 1447.20666452;
        double fy = 1445.36496334;
        double cx = 938.27422;
        double cy = 596.46596293;

        aprilTag = new AprilTagProcessor.Builder()
                //.setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(fx, fy, cx, cy)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }
    public void set_processor(){
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
    public void stop_stream(){
        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.close();
    }
    @SuppressLint("DefaultLocale")
    public void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x * 2.54 + shift[0],
                            detection.robotPose.getPosition().y * 2.54 + shift[1],
                            detection.robotPose.getPosition().z * 2.54 + shift[2]));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    telemetry.update();
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                telemetry.update();
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.update();
    }
    public double[] get_position(){
        double x = 0, y = 0, z = 0;
        double p = 0, r = 0, yaw = 0;
        double id = 0;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    x = detection.robotPose.getPosition().x * 2.54 + shift[0];
                    y = detection.robotPose.getPosition().y * 2.54 + shift[1];
                    z = detection.robotPose.getPosition().z * 2.54 + shift[2];
                    p = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
                    r = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
                    yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                    id = detection.id;
                } else {
                    id = detection.id;
                }
            }
        }
        double[] pos = new double[]{currentDetections.size(), x, y, z, p, r, yaw, id};
        return pos;
    }
    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                             VisionPortal visionPortal, AprilTagProcessor aprilTag,
                             LinearOpMode L){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
        this.visionPortal = visionPortal;
        this.aprilTag = aprilTag;
        initAprilTag();
    }
}
