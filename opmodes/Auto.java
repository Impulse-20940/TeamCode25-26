package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;

@Config
@Autonomous(name = "xy-moving")
public class Auto extends LinearOpMode {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    double axial;
    double lateral;
    double yaw;
    public static double x;
    public static double y;

    public static double x1;
    public static double y1;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        IMU imu = new IMU();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, null, cam, wheel, this);

        wheel.reset_encoders();

        waitForStart();

        double tic_per_cm = 31.4/480;
        double x1_1 = x1 / tic_per_cm;
        double x_1 = x / tic_per_cm;
        double y1_1 = y1 / tic_per_cm;
        double y_1 = y / tic_per_cm;

        double sx = x1_1 - x_1;
        double sy = y1_1 - y_1;

        double s = Math.sqrt(Math.pow(sx, 2) + Math.pow(sy, 2));

        while(wheel.get_enc_pos() < s && opModeIsActive()) {
            axial = sy/s;
            lateral = sx/s;
            yaw = imu.get_st_err(0, 0.012);

            double lfp  = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp   = axial - lateral + yaw;
            double rbp  = axial + lateral - yaw;

            wheel.setMPower(rbp, rfp, lfp, lbp);

            telemetry.addData("Now is (tics):", "%4f", wheel.get_enc_pos());
            telemetry.update();
        }
    }
}
