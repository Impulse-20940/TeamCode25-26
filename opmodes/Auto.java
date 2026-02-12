package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    MultipleTelemetry telemetry = new MultipleTelemetry();
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
        //********************************************** s0
        //______________________________________________ m1
        r.move_xy(0, 0, 0, -40, 0, 0.001, 0.02);                //s0m1a1
        while(cannon.get_shooter_vel() < 1600) cannon.fw.setPower(1);;
        cannon.fw_control_np(-1);
        //______________________________________________ m2
        cannon.bw_control(0);
        cannon.srv1_control(80);
        r.move_xy(0, 0, 0, -35 - 30, 0, 0.001, 0.02);          //s0m2a1
        r.delay(500);
        r.move_xy(0, 0, 0, 30, 0, 0.001, 0.02);                //s0m2a2
        r.delay(200);
        //______________________________________________

        //********************************************** s1
        if(r.alliance == 20){
            //__________________________________________ m1
            r.stable(0, 0, 45, 2000, 0.0105);                        //s1m1a1
            r.move_xy(0, -10, 0, 60, 45, 0.001, 0.02);         //s1m1a2
            cannon.bw_control(-1);
            r.move_xy(0, 0, 0, 20, 45, 0.001, 0.02);           //s1m1a3
            r.delay(500);
            cannon.bw_control(0);

            //__________________________________________ m2
            r.move_xy(0, 10, 0, -60, 45, 0.001, 0.02);        //s1m2a2
            r.turn(0, 0.01, 1200);                                         //s1m2a3
            r.delay(500);

            //__________________________________________ m3
            r.move_xy(0, 0, 0, 30, 0, 0.001, 0.02);           //s1m3a1
            while(cannon.get_shooter_vel() < 1600) cannon.fw.setPower(1);;
            cannon.fw_control_np(-1);
            cannon.bw_control(0);
            cannon.srv1_control(80);
            cam.stop_stream();
            r.move_xy(0, 10, 0, 0, 0, 0.001, 0.02);
        //********************************************** s2
        }else if (r.alliance == 24){
            r.stable(0, 0, -45, 2000, 0.009);
            r.move_xy(0, 15, 0, 30, -45, 0.001, 0.02);
            cannon.bw_control(-1);
            r.move_xy(0, 0, 0, -20, -45, 0.001, 0.02);
            r.delay(500);
            cannon.bw_control(0);

            //********После заброса
            r.move_xy(0, 0, 0, 10, -45, 0.001, 0.02);
            r.move_xy(0, 5, 0, 30, -45, 0.001, 0.02);
            r.turn(0, 0.012, 1400);
            r.delay(500);

            r.move_xy(0, 0, 0, 30, 0, 0.001, 0.02);
            while(cannon.get_shooter_vel() < 1600) cannon.fw.setPower(1);;
            cannon.fw_control_np(-1);
            cannon.bw_control(0);
            cannon.srv1_control(80);
            cam.stop_stream();
            r.move_xy(0, 10, 0, 0, 0, 0.001, 0.02);
        }
        //**********************************************
        cam.stop_stream();
    }
}
