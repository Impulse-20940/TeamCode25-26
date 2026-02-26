package org.firstinspires.ftc.teamcode.opmodes;
import android.opengl.Matrix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    boolean st = false, flag;
    double axial, lateral, yaw, min_speed = 1850;
    public static double kp;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        IMU imu = new IMU();
        Cannon cannon = new Cannon();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, null, cam, wheel, this);

        cam.set_processor();
        wheel.telemetry_ports();

        waitForStart();
        while (opModeIsActive()){
            double[] pos = cam.get_position(); 
            telemetry.addData("XYZ: ", "%4f, %4f, %4f", pos[1],
                                                                pos[2], pos[3]);

            double x =  gamepad1.left_stick_x * (1 - gamepad1.right_trigger);
            double y = -gamepad1.left_stick_y * (1 - gamepad1.right_trigger);

            double deg = imu.getTurnAngle();
            double l_alpha = 90 + deg;
            double a_alpha = 90 - deg;

            double a_rads = Math.toRadians(a_alpha);
            double l_rads = Math.toRadians(l_alpha);

            axial = x * Math.cos(l_rads) + y * Math.sin(a_rads);
            lateral = x * Math.sin(l_rads) + y * Math.cos(a_rads);

            boolean btn_a = gamepad1.a;
            if(btn_a){
                if(flag) imu.calibrate_imu();
                flag = false;
            } else flag = true;

            boolean btn_s = gamepad1.square;
            if(btn_s){
                if(flag){
                    st = !st;
                    flag = false;
                };
            } else flag = true;

            yaw = cam.get_position()[1] * kp; 

            telemetry.addData("Stable enabled: ", st);
            double lfp = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp = axial - lateral + yaw;
            double rbp = axial + lateral - yaw;

            wheel.setMPower(rbp, rfp, lfp, lbp);
            wheel.setZPB();
            telemetry.update();
        }
        cam.stop_stream();
    }

}
