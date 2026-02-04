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
@Autonomous(name="test_Autonomous")
public class test_auto extends LinearOpMode {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    @Override
    public void runOpMode() {
        RobotBuild R = new RobotBuild();
        IMU imu = new IMU();
        Wheelbase wheel = new Wheelbase();
        Camera cam = new Camera();
        Cannon cannon = new Cannon();
        R.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, cannon, cam, wheel, this);
        waitForStart();
        cannon.srv1_control(80);
        cannon.bw_control(1);
        R.move_xy(0, 0, 0, -40, 0, 1, 0.006, 0.08);
        cannon.bw_control(-1);
        R.delay(5);
        cannon.bw_control(0);
        while (true){
            cannon.fw_control(-1, 1600);
            if(cannon.get_srv_pos() == 0){
                R.delay(2000);
                break;
            }
        }
        cannon.srv1_control(120);
        cannon.bw_control(-1);
        R.delay(4000);
        while (true){
            cannon.fw_control(-1, 1600);
            if(cannon.get_srv_pos() == 0){
                R.delay(2000);
                break;
            }
        }
    }
}