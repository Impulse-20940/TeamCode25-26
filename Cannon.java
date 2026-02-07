package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cannon {
    ElapsedTime runtime = new ElapsedTime();
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;
    public DcMotorEx fw;
    public DcMotor bw;
    public Servo srv1;
    public boolean shoot;
    public double shoot_time;

    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                             LinearOpMode L){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
        this.fw = hardwareMap.get(DcMotorEx.class, "c1");
        this.bw = hardwareMap.get(DcMotor.class, "c2");
        this.srv1 = hardwareMap.get(Servo.class, "s1");
        //c2.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void fw_control(double power, double min_speed){
        fw.setPower(-power);
        boolean bmp_rt = gamepad2.right_bumper;
        telemetry.addData("Velocity: ", get_shooter_vel());
        for(int i = 0; i < 3; i += 1) {
            if (get_shooter_vel() > min_speed || bmp_rt) {
                if (!shoot) {
                    srv1_control(0);
                    shoot_time = runtime.milliseconds();
                    shoot = true;
                }
                if (runtime.milliseconds() - shoot_time > 1000) {
                    srv1_control(80);
                    telemetry.addData("Down", 0);
                }
                telemetry.addData("Runtime ", runtime.milliseconds() - shoot_time);
            } else {
                srv1_control(80);
                shoot = false;
            }
            telemetry.update();
            Thread.yield();
        }
    }
    public void bw_control(double power){
        bw.setPower(-power);
    }
    public void srv1_control(double pos){
        srv1.setPosition(pos);
    }
    public double get_shooter_vel(){
        return fw.getVelocity();
    }
    public double get_srv_pos(){
        return srv1.getPosition();
    }
}