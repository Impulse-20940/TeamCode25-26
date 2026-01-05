package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public DcMotorEx fw;
    public DcMotor bw;
    public Servo srv1;

    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                             DcMotorEx FW, DcMotor BW,
                             LinearOpMode L, Servo SRV1){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
        this.fw = FW;
        this.bw = BW;
        this.srv1 = SRV1;
        this.fw = hardwareMap.get(DcMotorEx.class, "c1");
        this.bw = hardwareMap.get(DcMotor.class, "c2");
        this.srv1 = hardwareMap.get(Servo.class, "s1");
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
    public double get_shooter_vel(){
        return fw.getVelocity();
    }
}