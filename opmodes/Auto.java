package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;

@Config
@Autonomous(name = "xy-moving")
public class Auto extends LinearOpMode {
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
        waitForStart();
        double sx = x1 - x;
        double sy = y1 - y;

        double s = Math.sqrt(Math.pow(sx, 2) + Math.pow(sy, 2));

        while(r.lf.getCurrentPosition() < s && opModeIsActive()) {
            axial = sy/s;
            lateral = sx/s;
            yaw = imu.get_st_err(0, 0.012);

            double lfp  = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp   = axial - lateral + yaw;
            double rbp  = axial + lateral - yaw;

            wheel.setMPower(rbp, rfp, lfp, lbp);
        }
    }
}
