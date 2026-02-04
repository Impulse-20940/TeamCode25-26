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
    double angle_m;
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
        double cnt = 0;
        waitForStart();
        while (opModeIsActive()) {
            double[] pos = cam.get_position();
            if(pos[6] != 0 && cnt < 10){
                angle_m += pos[6];
                cnt += 1;
            } else if (cnt == 10){
                cnt = 0;
                angle = angle_m / 10;
                angle_m = 0;
            }
            telemetry.addData("Now is ", -45 + angle);
            telemetry.addData("Position is ", wheel.get_enc_pos());
            telemetry.addData("Position_left is ", wheel.get_enc_pos_res());
            telemetry.update();
            if(angle > 0){
                double yaw = (-45+angle) * 0.01;
                //Вычисление мощности
                double lfp = (+yaw);
                double rfp = (-yaw);
                double lbp = (+yaw);
                double rbp = (-yaw);

                wheel.setMPower(rbp, rfp, lfp, lbp);
            }
        }
        cam.stop_stream();

    }

}
