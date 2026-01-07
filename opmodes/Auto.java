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
    //Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        RobotBuild r = new RobotBuild();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        Cannon cannon = new Cannon();
        IMU imu = new IMU();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, cannon, cam, wheel, this);

        wheel.reset_encoders();
        waitForStart();

        cam.set_processor();
        r.move_xy(0, 0, 0, -35, 0, 1, 0.014);

        runtime.reset();
        while (true){
            cannon.fw_control(-1, 1500); //1600
            cannon.bw_control(-1);
            if(cannon.get_srv_pos() == 0){
                r.delay(2000);
                break;
            }
        }
        cannon.bw_control(0);
        cannon.fw_control(0, 1600);

        cam.telemetryAprilTag();
        cam.stop_stream();
    }
}
