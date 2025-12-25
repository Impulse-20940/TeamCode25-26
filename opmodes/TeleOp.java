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
    boolean st90;
    boolean flag;
    double axial;
    double lateral;
    double yaw;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        IMU imu = new IMU();
        Cannon cannon = new Cannon();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, null, null, wheel, this);
        cam.set_processor();
        wheel.telemetry_ports();

        waitForStart();
        while(opModeIsActive()){
            cam.telemetryAprilTag();
            /*
            double bmp_rt = gamepad2.right_trigger;
            if(bmp_rt > 0.5){
                cannon.srv1_control(80);
            } else cannon.srv1_control(15);
            // 2 - серво-мотор
            double bmp_lt = gamepad2.left_trigger;
            if(bmp_lt > 0.5){
                cannon.srv2_control(90);
            } else cannon.srv2_control(0);
            //Нажата кнопка B - стабилизация 90 грд
            */
            boolean btn_b = gamepad1.b;
            if(btn_b){
                if(flag) st90 = !st90;
                flag = false;
            } else flag = true;

            boolean btn_x = gamepad1.x;
            if(btn_x){
                if(flag) imu.calibrate_imu();
                flag = false;
            } else flag = true;

            // Проверка стабизизации
            if(st90){
                axial = gamepad1.left_stick_x;
                lateral = -gamepad1.left_stick_y;
                yaw = imu.get_st_err(-90, 0.012);
            } else { //without head
                double x = -gamepad1.left_stick_x;
                double y = gamepad1.left_stick_y;

                double deg = imu.getTurnAngle();
                double l_alpha = 90 + deg;
                double a_alpha = 90 - deg;

                double a_rads = Math.toRadians(a_alpha);
                double l_rads = Math.toRadians(l_alpha);

                axial = x * Math.cos(l_rads) + y * Math.sin(a_rads);
                lateral = x * Math.sin(l_rads) + y * Math.cos(a_rads);
                yaw = -gamepad1.right_stick_x;

                //telemetry.addData("Axial is", axial);
                //telemetry.addData("Lateral is", lateral);
                //telemetry.addData("Yaw is", yaw);
           }
            double lfp = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp = axial - lateral + yaw;
            double rbp = axial + lateral - yaw;

            //cannon.fw_control(gamepad2.left_bumper? 1 : 0);
            //cannon.bw_control(gamepad1.right_trigger);

            wheel.setMPower(rbp, rfp, lfp, lbp);
            wheel.setZPB();

            //wheel.telemetry_power();
        }
        cam.stop_stream();
    }
}
