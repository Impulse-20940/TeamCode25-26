package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotBuild extends Robot{
    public void init(HardwareMap hardwareMap, Telemetry telemetry,
                     Gamepad gamepad1, Gamepad gamepad2, IMU Imu, Cannon cnn,
                     Camera cam, Wheelbase wheel, LinearOpMode L){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
        this.runtime = new ElapsedTime();
        if (Imu != null){
            this.Imu = Imu;
            Imu.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.imu, L);
        }
        if (cnn != null){
            this.cnn = cnn;
            cnn.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.c1, this.c2, L);
        }
        if (cam != null){
            this.cam = cam;
            cam.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.visionPortal, this.aprilTag, L);
        }

        if (wheel != null){
            this.wb = wheel;
            wheel.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.rf, this.rb, this.lf, this.lb, L);
        }
    }
    public void stable(double a, double l, double stable, long time, double kt){
        runtime.reset();
        while(L.opModeIsActive() && runtime.milliseconds() < time){
            double axial = a;
            double lateral = l;
            double yaw = Imu.get_st_err(stable, kt);

            double lfp  = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp   = axial - lateral + yaw;
            double rbp  = axial + lateral - yaw;

            wb.setMPower(rbp, rfp, lfp, lbp);
        }
        wb.setMPower(0, 0, 0, 0);
        wb.setZPB();
    }
    void stable180(double a, double l, double stable, long time, double kt){
        runtime.reset();
        double getangle = 0;
        while(L.opModeIsActive() && runtime.milliseconds() < time){
            if (Imu.getTurnAngle() > 0){
                getangle = stable-Imu.getTurnAngle();
            }
            if (Imu.getTurnAngle() < 0){
                getangle = -stable-Imu.getTurnAngle();
            }
            double axial = a;
            double lateral = l;
            double yaw = getangle*kt;

            double lfp  = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp   = axial - lateral + yaw;
            double rbp  = axial + lateral - yaw;

            wb.setMPower(rbp, rfp, lfp, lbp);
        }
        wb.setMPower(0, 0, 0, 0);
        wb.setZPB();
    }
}