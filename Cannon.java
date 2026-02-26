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

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

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
    public double shoot_time;
    public boolean shoot, get_third, vibro;

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
    public void fw_control(double power, double min_speed) {
        fw.setPower(-power);
        boolean bmp_lt = gamepad2.left_bumper;
        boolean bmp_rt = gamepad2.right_bumper;
        if(get_shooter_vel() > min_speed){
            gamepad2.rumble(200);
            vibro = false;
        }
        if (bmp_lt) {
            if (!shoot) {
                ExecutorService executor = Executors.newCachedThreadPool();
                executor.execute(() -> {
                    for (int i = 0; i < 3; i += 1) {
                        if (get_third) {
                            runtime.reset();
                            while (runtime.milliseconds() < 195) bw.setPower(1);
                            bw.setPower(-1);
                            runtime.reset();
                            while (runtime.milliseconds() < 1000) bw.setPower(-1);
                            get_third = false;
                        } ;

                        srv1_control(0);
                        runtime.reset();
                        while (runtime.milliseconds() < 700) ;
                        srv1_control(80);
                        runtime.reset();
                        while (runtime.milliseconds() < 50) bw.setPower(-1);
                        runtime.reset();
                        while (runtime.milliseconds() < 1000) ;
                        if (i == 1) get_third = true;
                    }
                });
                shoot = true;
                bw.setPower(0);
                fw.setPower(0);
            }
        } else if(bmp_rt){
            if(!shoot){
                srv1_control(0);
                shoot_time = runtime.milliseconds();
                shoot = true;
            }
            if(runtime.milliseconds()-shoot_time > 1000){
                srv1_control(80);
            }
        } else {
            srv1_control(80);
            shoot = false;
            vibro = true;
        }
    }
    public void fw_control_np(double power) {
        fw.setPower(-power);
        for (int i = 0; i < 3; i += 1) {
            if (get_third) {
                runtime.reset();
                while (runtime.milliseconds() < 160) bw.setPower(1);
                bw.setPower(-1);
                runtime.reset();
                while (runtime.milliseconds() < 1000) bw.setPower(-1);
                get_third = false;
            } ;

            srv1_control(0);
            runtime.reset();
            while (runtime.milliseconds() < 700) ;
            srv1_control(80);
            runtime.reset();
            while (runtime.milliseconds() < 1000) ;
            while (runtime.milliseconds() < 20) bw.setPower(-1);
            if (i == 1) get_third = true;
        }
        fw.setPower(0);
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
}