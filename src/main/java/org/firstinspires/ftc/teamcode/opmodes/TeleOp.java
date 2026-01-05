package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Main_TeleOp")
public class TeleOp extends LinearOpMode {
    //Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    boolean st90;
    boolean st0;
    boolean st45_lt;
    boolean st45_rt;
    boolean flag;
    double axial;
    double lateral;
    double yaw;
    public static double a;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        IMU imu = new IMU();
        //Cannon cannon = new Cannon();
        //Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, null, null, wheel, this);
        //cam.set_processor();
        waitForStart();
        while(opModeIsActive()){
            //cam.telemetryAprilTag();
            // 1 - серво-мотор
            /* double bmp_rt2 = gamepad2.right_trigger;
            if(bmp_rt2 > 0.5){
                cannon.srv1_control(80);
            } else cannon.srv1_control(15);
            // 2 - серво-мотор
            double bmp_lt2 = gamepad2.left_trigger;
            if(bmp_lt2 > 0.5){
                cannon.srv2_control(90);
            } else cannon.srv2_control(0); */

            double x =  -gamepad1.left_stick_x;
            double y =  gamepad1.left_stick_y;

            double deg = imu.getTurnAngle();
            double l_alpha = 90 + deg;
            double a_alpha = 90 - deg;

            double a_rads = Math.toRadians(a_alpha);
            double l_rads = Math.toRadians(l_alpha);

            axial = x * Math.cos(l_rads) + y * Math.sin(a_rads);
            lateral = x * Math.sin(l_rads) + y * Math.cos(a_rads);

            // Нажат левый бампер - стабилизация 45 градусов влево
            boolean bmp_lt = gamepad1.left_bumper;
            if(bmp_lt){
                if(flag){
                    st45_lt = !st45_lt;
                    st90 = false;
                    st45_rt = false;
                    st0 = false;
                }
                flag = false;
            } else flag = true;

            // Нажат правый бампер - стабилизация 45 градусов вправо
            boolean bmp_rt = gamepad1.right_bumper;
            if(bmp_rt){
                if(flag){
                    st45_lt = false;
                    st90 = false;
                    st45_rt = !st45_rt;
                    st0 = false;
                }
                flag = false;
            } else flag = true;

            // Нажата кнопка Y - стабилизация 0 градусов
            boolean btn_y = gamepad1.y;
            if(btn_y){
                if(flag){
                    st45_lt = false;
                    st90 = false;
                    st45_rt = false;
                    st0 = !st0;
                }
                flag = false;
            } else flag = true;

            // Нажата кнопка B - стабилизация 90 градусов
            boolean btn_b = gamepad1.b;
            if(btn_b){
                if(flag) {
                    st45_lt = false;
                    st90 = !st90;
                    st45_rt = false;
                    st0 = false;
                }
                flag = false;
            } else flag = true;

            // Проверка стабизизации
            if(st90) {  // 90 angle
                yaw = imu.get_st_err(-90, 0.012);
            } else if(st45_lt){  // 45 angle left
                yaw = imu.get_st_err(-45, 0.012);
            } else if(st45_rt){  // 45 angle right
                yaw = imu.get_st_err(45, 0.012);
            } else if(st0){  // 0 angle
                yaw = imu.get_st_err(0, 0.012);
            } else {
                yaw = -gamepad1.right_stick_x;
                telemetry.addData("Axial is", axial);
                telemetry.addData("Lateral is", lateral);
                telemetry.addData("Yaw is", yaw);
           }
            double lfp = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp = axial - lateral + yaw;
            double rbp = axial + lateral - yaw;
             
            //cannon.fw_control(gamepad1.left_bumper? 1 : 0);
            //cannon.bw_control(gamepad2.right_trigger);

            wheel.setMPower(rbp, rfp, lfp, lbp);
            wheel.setZPB();
        }
        //cam.stop_stream();
    }
}
