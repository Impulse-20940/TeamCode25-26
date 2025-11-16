package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;
import java.lang.Math;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Main_TeleOp")
public class TeleOp extends LinearOpMode {
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
                gamepad2, imu, null, cam, wheel, this);
        //cam.set_processor();
        waitForStart();
        while(opModeIsActive()){
            cam.telemetryAprilTag();
            //Нажата кнопка B - стабилизация 90 грд
//            boolean btn_b = gamepad1.b;
//            if(btn_b){
//                if(flag) st90 = !st90;
//                flag = false;
//            } else flag = true;
//            // Проверка стабизизации
//            if(st90){
//                axial = -gamepad1.left_stick_x;
//                lateral = -gamepad1.left_stick_y;
//                yaw = imu.get_st_err(-90, 0.012);
//            } else {
//                axial = gamepad1.left_stick_y;
//                lateral = -gamepad1.left_stick_x;
//                yaw = -gamepad1.right_stick_x;
//            }
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double deg = imu.getTurnAngle();
            double rar = Math.toRadians(deg);
            double cos = Math.cos(rar);
            double sin = Math.sin(rar);

            double axial = y * cos + x * sin;
            double lateral = -y * sin + x * cos;

            double lfp = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp = axial - lateral + yaw;
            double rbp = axial + lateral - yaw;

            wheel.setMPower(rbp, rfp, lfp, lbp);
            wheel.setZPB();

            cannon.fw_control(gamepad1.right_bumper? 1 : 0);
            cannon.bw_control(gamepad1.right_trigger);
        }
        cam.stop_stream();
    }
}
