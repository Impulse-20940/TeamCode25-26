package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cannon {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;
    public DcMotor fw;
    public DcMotor bw;

    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                             DcMotor FW, DcMotor BW,
                             LinearOpMode L){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
        this.fw = FW;
        this.bw = BW;
        fw = hardwareMap.get(DcMotor.class, "c1");
        bw = hardwareMap.get(DcMotor.class, "c2");
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
}
