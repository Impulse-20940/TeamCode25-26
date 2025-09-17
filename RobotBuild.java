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
        Imu.init_classes(hardwareMap, telemetry, gamepad1, gamepad2, L);
        cnn.init_classes(hardwareMap, telemetry, gamepad1, gamepad2, L);
        cam.init_classes(hardwareMap, telemetry, gamepad1, gamepad2, L);
        wheel.init_classes(hardwareMap, telemetry, gamepad1, gamepad2, L);
        Imu.init(this.imu);
        cnn.init(this.cannon);
        cam.init(this.visionPortal, this.aprilTag);
        wheel.init(this.rf, this.rb, this.lf, this.lb);
    }
}