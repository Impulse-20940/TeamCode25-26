package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
        if (Imu != null){
            Imu.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.imu, L);
            Imu.init(cnn, cam, wheel);
        }
        if (cnn != null){
            cnn.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.cannon, L);
            cnn.init(Imu, cam, wheel);
        }
        if (cam != null){
            cam.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.visionPortal, this.aprilTag, L);
            cam.init(Imu, cnn, wheel);
        }

        if (wheel != null){
            wheel.init_classes(hardwareMap, telemetry, gamepad1, gamepad2,
                    this.rf, this.rb, this.lf, this.lb, L);
            wheel.init(Imu, cnn, cam);
        }
    }
}