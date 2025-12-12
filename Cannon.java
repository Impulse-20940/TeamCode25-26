package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cannon {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;
    public DcMotor fw;
    public DcMotor bw;
    public Servo srv1;
    public Servo srv2;

    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                             DcMotor FW, DcMotor BW,
                             LinearOpMode L, Servo SRV1, Servo SRV2){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
        this.fw = FW;
        this.bw = BW;
        this.srv1 = SRV1;
        this.srv2 = SRV2;
        fw = hardwareMap.get(DcMotor.class, "c1");
        bw = hardwareMap.get(DcMotor.class, "c2");
        srv1 = hardwareMap.get(Servo.class, "s1");
        srv2 = hardwareMap.get(Servo.class, "s2");
        //c2.setDirection(DcMotorSimple.Direction.REVERSE);
    }
//    public void cannon_control(double power){
//        c2.setPower(power);
//    }
    public void fw_control(double power){
        fw.setPower(power);
    }
    public void bw_control(double power){
        bw.setPower(power);
    }
    public void srv1_control(double pos){
        srv1.setPosition(pos);
    }
    public void srv2_control(double pos){
        srv2.setPosition(pos);
    }
}