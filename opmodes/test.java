package org.firstinspires.ftc.teamcode.opmodes;
import android.opengl.Matrix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class test extends LinearOpMode {
    boolean turn90;
    boolean st90;
    boolean flag;
    double axial;
    double lateral;
    double yaw;
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
            //Нажата кнопка B - стабилизация 90 грд

            if(gamepad1.a){
                if(flag) turn90 = !turn90;
                flag = false;
            } else flag = true;

            boolean btn_b = gamepad1.b;
            if(btn_b){
                if(flag) st90 = !st90;
                flag = false;
            } else flag = true;
            // Проверка стабизизации
            if(st90){
                axial = -gamepad1.left_stick_x;
                lateral = -gamepad1.left_stick_y;
                yaw = imu.get_st_err(-90, 0.012);
            } else {
                axial = gamepad1.left_stick_y;
                lateral = -gamepad1.left_stick_x;
                yaw = -gamepad1.right_stick_x;
            }
            // Базовое управление колёсной базой
            double lfp = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp = axial - lateral + yaw;
            double rbp = axial + lateral - yaw;

            wheel.setMPower(rbp, rfp, lfp, lbp);
            wheel.setZPB();

            r.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        //cam.stop_stream();
}

