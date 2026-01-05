package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;
import java.lang.Math;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Main_TeleOp")
public class TeleOp extends LinearOpMode {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    boolean st45_lt, st45_rt, flag;
    double axial, lateral, yaw;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        IMU imu = new IMU();
        Cannon cannon = new Cannon();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, cannon, cam, wheel, this);

        wheel.telemetry_ports();
        cam.set_processor();
        cannon.srv1_control(80);

        waitForStart();
        while(opModeIsActive()){
            //cam.telemetryAprilTag();

            boolean bmp_rt = gamepad2.right_bumper;
            if((cannon.get_shooter_vel() > 0.95 || bmp_rt) && flag){
                r.delay(1000);
                cannon.srv1_control(0);
                flag = false;
            } else{
                cannon.srv1_control(80);
                flag = true;
            }
            //Нажата кнопка B - стабилизация 90 грд

            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;

            double deg = imu.getTurnAngle();
            double l_alpha = 90 + deg;
            double a_alpha = 90 - deg;

            double a_rads = Math.toRadians(a_alpha);
            double l_rads = Math.toRadians(l_alpha);

            axial = x * Math.cos(l_rads) + y * Math.sin(a_rads);
            lateral = x * Math.sin(l_rads) + y * Math.cos(a_rads);

            boolean btn_b = gamepad1.b;
            if(btn_b){
                if(flag){
                    st45_rt = !st45_rt;
                    st45_lt = false;
                }
                flag = false;
            } else flag = true;

            boolean btn_x = gamepad1.x;
            if(btn_x){
                if(flag){
                    st45_lt = !st45_lt;
                    st45_rt = false;
                }
                flag = false;
            } else flag = true;

            boolean btn_a = gamepad1.a;
            if(btn_a){
                if(flag) imu.calibrate_imu();
                flag = false;
            } else flag = true;

            // Проверка стабизизации
            if(st45_lt){
                yaw = imu.get_st_err(-45, 0.012);
            } else if (st45_rt){
                yaw = imu.get_st_err(45, 0.012);
            }else { //without head
                yaw = gamepad1.right_stick_x;

                telemetry.addData("Axial is", axial);
                telemetry.addData("Lateral is", lateral);
                telemetry.addData("Yaw is", yaw);
                telemetry.addData("Shooter velocity", cannon.get_shooter_vel());
           }
            double lfp = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp = axial - lateral + yaw;
            double rbp = axial + lateral - yaw;

            cannon.fw_control(gamepad2.left_stick_y);
            cannon.bw_control(gamepad2.right_stick_y);

            wheel.setMPower(rbp, rfp, lfp, lbp);
            wheel.setZPB();
            telemetry.update();
            //wheel.telemetry_power();
        }
        cam.stop_stream();
    }
}
