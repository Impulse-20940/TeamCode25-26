package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotBuild extends Robot {
    public void init(HardwareMap hardwareMap, Telemetry telemetry,
                     Gamepad gamepad1, Gamepad gamepad2, IMU Imu, Cannon cnn,
                     Camera cam, Wheelbase wheel, LinearOpMode L) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
        this.runtime = new ElapsedTime();
        if (Imu != null) {
            this.Imu = Imu;
            Imu.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.imu, L);
        }
        if (cnn != null) {
            this.cnn = cnn;
            cnn.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.fw, this.c2, L, srv1, srv2);
        }
        if (cam != null) {
            this.cam = cam;
            cam.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.visionPortal, this.aprilTag, L);
        }

        if (wheel != null) {
            this.wb = wheel;
            wheel.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.rf, this.rb, this.lf, this.lb, L);
        }
    }

    public void stable(double a, double l, double stable, long time, double kt) {
        runtime.reset();
        while (L.opModeIsActive() && runtime.milliseconds() < time) {
            double getangle = stable - Imu.getTurnAngle();
            double axial = a;
            double lateral = l;
            double yaw = getangle * kt;

            double lfp = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp = axial - lateral + yaw;
            double rbp = axial + lateral - yaw;

            wb.setMPower(rbp, rfp, lfp, lbp);
        }
        wb.setMPower(0, 0, 0, 0);
        wb.setZPB();
    }

    void stable180(double a, double l, double stable, long time, double kt) {
        runtime.reset();
        double getangle = 0;
        while (L.opModeIsActive() && runtime.milliseconds() < time) {
            if (Imu.getTurnAngle() > 0) {
                getangle = stable - Imu.getTurnAngle();
            }
            if (Imu.getTurnAngle() < 0) {
                getangle = -stable - Imu.getTurnAngle();
            }
            double axial = a;
            double lateral = l;
            double yaw = getangle * kt;

            double lfp = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp = axial - lateral + yaw;
            double rbp = axial + lateral - yaw;

            wb.setMPower(rbp, rfp, lfp, lbp);
        }
        wb.setMPower(0, 0, 0, 0);
        wb.setZPB();
    }

    public void turn(double grd, double kt) {//Функция поворота
        double yaw;
        runtime.reset();

        while (L.opModeIsActive() && runtime.milliseconds() < 1000) {
            //Вычисление угла стабилизации
            yaw = Imu.get_st_err(grd, kt);
            //Вычисление мощности
            double lfp = (+yaw);
            double rfp = (-yaw);
            double lbp = (+yaw);
            double rbp = (-yaw);
            double grd_tel = Imu.getTurnAngle();

            wb.setMPower(rbp, rfp, lfp, lbp);
            telemetry.addData("Now is (degrees):", "%4f", grd_tel);
            telemetry.update();
        }
        wb.setZPB();
    }

    public void fd(double cm, double kp){//Функция проезда вперёд
        wb.reset_encoders();
        double tic_per_cm = (12.36/480)*2.54;//коэф перевода из тиков в сантиметры
        double ang        = Imu.getTurnAngle();
        double axial = 1;
        while (Math.abs(wb.get_enc_pos())*tic_per_cm*1.4 < cm){
            double err = cm-(wb.get_enc_pos()*tic_per_cm);//Формирование ошибки

            double p = err * kp * -1;           //Коэф пропорциональности
            double yaw   = Imu.get_st_err(ang, 0.012);

            double lfp = (axial+yaw)*p;    //Формирование выходных значений
            double rfp = (axial-yaw)*p;
            double lbp = (axial+yaw)*p;
            double rbp = (axial-yaw)*p;
            telemetry.addData("Encoder is", wb.get_enc_pos());
            telemetry.update();
            stable(0, 0, ang, 100, 0.012);
            wb.setMPower(rbp, rfp, lfp, lbp);
        }
        wb.setMPower(0, 0, 0, 0);
        wb.setZPB();
        stable(0, 0, ang, 1000, 0.012);
    }




    public void move_xy(double x, double x1, double y, double y1, double angle, double kt){
        wb.reset_encoders();
        double tic_per_inch  = 12.36/480;
        x1                  /= tic_per_inch;
        x                   /= tic_per_inch;
        y1                  /= tic_per_inch;
        y                   /= tic_per_inch;

        double sx            = x1 - x;
        double sy            = y1 - y;

        double s = Math.sqrt(Math.pow(sx, 2) + Math.pow(sy, 2));

        while(Math.abs(wb.get_enc_pos()) < s && L.opModeIsActive()) {

            double axial    = sy/s;
            double lateral  = sx/s;
            //double yaw      = Imu.get_st_err(angle, kt);
            double yaw = 0;

            double lfp      = axial + lateral + yaw;
            double rfp      = axial - lateral - yaw;
            double lbp      = axial - lateral + yaw;
            double rbp      = axial + lateral - yaw;

            wb.setMPower(rbp, rfp, lfp, lbp);

            telemetry.addData("Now is (tics):", "%4f, %4f", Math.abs(wb.get_enc_pos()),
                                                                                                    s);
            telemetry.addData("Angle is", "%4f, needs %4f",
                                                            Imu.getTurnAngle(), angle);
            telemetry.addData("Stable error", "%4f",
                                                Imu.get_st_err(angle, kt));
            telemetry.update();
        }
    }
}