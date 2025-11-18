package org.firstinspires.ftc.teamcode.opmodes;
import android.opengl.Matrix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Main_TeleOp")
public class TeleOp extends LinearOpMode {
    boolean st, st90, flag;
    double axial, lateral, yaw;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        IMU imu = new IMU();
        //Cannon cannon = new Cannon();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, null, cam, wheel, this);
        //cam.set_processor();
        waitForStart();
        while(opModeIsActive()){
            //cam.telemetryAprilTag();
            //Нажата кнопка B - стабилизация 90 грд
            boolean btn_b = gamepad1.b,
                    btn_y = gamepad1.y;
            if(btn_b){
                if(flag) st90   = !st90;
                if(st)     st   = false;
                flag            = false;
            } else flag = true;
            if(btn_y){
                if(flag) st     = !st;
                if(st90) st90   = false;
                flag            = false;
            } else flag = true;
            // Проверка стабизизации
            if(st90){
                axial   = -gamepad1.left_stick_x;
                lateral = -gamepad1.left_stick_y;
                yaw     = imu.get_st_err(-90, 0.012);
            } else if(st){
                axial   = gamepad1.left_stick_y;
                lateral = -gamepad1.left_stick_x;
                yaw     = imu.get_st_err(0, 0.012);
            } else {
                axial   = gamepad1.left_stick_y;
                lateral = -gamepad1.left_stick_x;
                yaw     = -gamepad1.right_stick_x;
            }
            double lfp = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp = axial - lateral + yaw;
            double rbp = axial + lateral - yaw;

            wheel.setMPower(rbp, rfp, lfp, lbp);
            wheel.setZPB();
        }
        //cam.stop_stream();
    }
}
