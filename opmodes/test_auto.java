package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;
@Config
@Autonomous(name="test_Autonomous")
public class test_auto extends LinearOpMode {
    public static double kp;
    public static double ki;
    public static double kd;
    public static double kt;
    MultipleTelemetry multiple_telemetry = new MultipleTelemetry(telemetry,
                    FtcDashboard.getInstance().getTelemetry());
    @Override
    public void runOpMode() {
        RobotBuild r = new RobotBuild();
        IMU imu = new IMU();
        Wheelbase wheel = new Wheelbase();
        Camera cam = new Camera();
        Cannon cannon = new Cannon();
        r.init(hardwareMap, multiple_telemetry, gamepad1,
                gamepad2, imu, cannon, null, null, this);
        while(opModeIsActive()){
            cannon.ShooterPID_sync(1, 1600, kp, ki, kd);
        }
    }
}