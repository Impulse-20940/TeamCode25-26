package org.firstinspires.ftc.teamcode.opmodes;
import android.opengl.Matrix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="test_TeleOp")
public class test extends LinearOpMode {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    double angle;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        IMU imu = new IMU();
        Cannon cannon = new Cannon();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, cannon, cam, wheel, this);

        cam.set_processor();
        wheel.telemetry_ports();
        wheel.reset_encoders();

        waitForStart();
        while (opModeIsActive()) {
            double[] pos = cam.get_position();
            if(pos[6] != 0){
                angle = pos[6] - 135;
            }
            telemetry.addData("Now is ", angle);
            telemetry.addData("Position is ", wheel.get_enc_pos());
            telemetry.addData("Position_left is ", wheel.get_enc_pos_res());
            telemetry.update();
            r.turn(angle, 0.012, 1);
        }
        cam.stop_stream();

    }

}
