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
    public DcMotor c1;
    public DcMotor c2;

    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                             DcMotor C1, DcMotor C2,
                             LinearOpMode L){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
        this.c1 = C1;
        this.c2 = C2;
        c1 = hardwareMap.get(DcMotor.class, "c1");
        c2 = hardwareMap.get(DcMotor.class, "c2");
    }
    public void cannon_control(double power){
        c1.setPower(power);
        c2.setPower(-power);
    }
}
