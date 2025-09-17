package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cannon {
    IMU gyro;
    Camera camera;
    Wheelbase wb;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;
    DcMotor cnn;

    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                             IMU Imu, Camera cam, Wheelbase wheel,
                             LinearOpMode L){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.gyro = Imu;
        this.camera = cam;
        this.wb = wheel;
        this.L = L;
    }
    public void init(DcMotor cannon){
        this.cnn = cannon;
        cnn = hardwareMap.get(DcMotor.class, "cnn");
    }
}
