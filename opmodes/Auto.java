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
        r.move_xy(0, 0, 0, -40, 0, 0, 0.006, 0.08);

        runtime.reset();
        while (true){
            cannon.fw_control(-1, 1600);
            cannon.bw_control(-1);
            if(cannon.get_srv_pos() == 0){
                r.delay(2000);
                break;
            }
        }
        cannon.bw_control(0);
        cannon.fw_control(0, 1600);
        cannon.srv1_control(120);
        r.move_xy(0, 0, 0, -35 - 40, 0, 1, 0.004, 0.18); //-35
        r.delay(500);
        r.move_xy(0, 0, 0, 40, 0, 1, 0.005, 0.18); //-35
        r.delay(500);

        if(r.alliance == 20){// Определение альянса
            r.stable(0, 0, -135, 2000, 0.0027);
            r.move_xy(0, 15, 0, -30, -135, 1, 0.004, 0.18); //-10
            cannon.bw_control(-1);
            r.move_xy(0, 0, 0, -20, -135, 1, 0.004, 0.18);
            r.delay(500);
            cannon.bw_control(0);

            //********После заброса
            r.move_xy(0, 0, 0, 10, -135, 1, 0.004, 0.18);
            r.move_xy(0, -42, 0, 42, -135, 1, 0.004, 0.18); //-10
            r.turn(0, 0.007, 1200);
            //r.delay(500);

            r.move_xy(0, 0, 0, 30, 0, 1, 0.004, 0.18); //-35
            runtime.reset();
            while (true){
                cannon.fw_control(-1, 1600);
                cannon.bw_control(-1);
                if(cannon.get_srv_pos() == 0){
                    r.delay(2000);
                    break;
                }
            }
            cannon.bw_control(0);
            cannon.fw_control(0, 1600);
            cannon.srv1_control(80);
            cam.stop_stream();
        }else if (r.alliance == 24){
            r.stable(0, 0, 135, 2000, 0.0027);
            r.move_xy(0, -15, 0, -30, 135, 1, 0.004, 0.18); //-10
            cannon.bw_control(-1);
            r.move_xy(0, 0, 0, -20, 135, 1, 0.004, 0.18);
            r.delay(500);
            cannon.bw_control(0);

            //********После заброса
            r.move_xy(0, 0, 0, 10, 135, 1, 0.004, 0.18);
            r.move_xy(0, 5, 0, 30, 135, 1, 0.004, 0.18); //-10
            r.turn(10, 0.012, 1400);
            //r.delay(500);

            r.move_xy(0, 0, 0, 30, 10, 1, 0.004, 0.18); //-35
            runtime.reset();
            while (true){
                cannon.fw_control(-1, 1600);
                cannon.bw_control(-1);
                if(cannon.get_srv_pos() == 0){
                    r.delay(2000);
                    break;
                }
            }
            cannon.bw_control(0);
            cannon.fw_control(0, 1600);
            cannon.srv1_control(80);
            cam.stop_stream();

        }
        cam.stop_stream();
    }
}
