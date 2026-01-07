package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;

@Config
@Autonomous(name = "Main_Autonomous")
public class Auto extends LinearOpMode {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    ElapsedTime runtime;
    public static double x;
    public static double y;

    public static double x1;
    public static double y1;
    @Override
    public void runOpMode() {
        RobotBuild r = new RobotBuild();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        Cannon cannon = new Cannon();
        IMU imu = new IMU();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, null, cam, wheel, this);

        wheel.reset_encoders();
        waitForStart();

        cam.set_processor();
        r.move_xy(0, 0, 0, 60, 0, 1, 0.006);

        runtime.reset();
        while (runtime.milliseconds() < 4000){
            cannon.fw_control(1, 1600);
        }
        cannon.fw_control(0, 1600);

        cam.telemetryAprilTag();
        cam.stop_stream();
    }
}
