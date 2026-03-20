package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class Cannon {
    ExecutorService shooter_thread = Executors.newCachedThreadPool();
    ExecutorService shooting_process = Executors.newCachedThreadPool();
    ElapsedTime runtime = new ElapsedTime();
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;
    public DcMotorEx fw;
    public DcMotor bw;
    public Servo srv1;
    public boolean value = false, shoot_value = false, get_third, vibro;
    double old_t = runtime.milliseconds();
    double err_last = 0, integral = 0, D;

    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                             LinearOpMode L){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
        this.fw = hardwareMap.get(DcMotorEx.class, "c1");
        this.bw = hardwareMap.get(DcMotor.class, "c2");
        this.srv1 = hardwareMap.get(Servo.class, "shoot");
        //fw.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void fw_control(double min_speed) {
        shooting_process.execute(() -> {
            while(L.opModeIsActive()){
                while(shoot_value) {
                    srv1_control(0.48);
                    runtime.reset();
                    while(runtime.milliseconds() < 500);
                    srv1_control(0.9);
                    while(runtime.milliseconds() < 1000);
                }
                srv1_control(0.9);
            }
        });
    }
    public void fw_control_np(double power) {
        fw.setPower(-power);
        for (int i = 0; i < 3; i += 1) {
            srv1_control(0.48);
            runtime.reset();
            while(runtime.milliseconds() < 500);
            srv1_control(0.9);
            while(runtime.milliseconds() < 1000);
        }
        fw.setPower(0);
    }
    public void ShooterPID_sync(double power, double speed, double kP, double kI, double kD){
        double err = speed - get_shooter_vel();
        double now = runtime.milliseconds();

        double dt = (now - old_t) / 1000.0;
        integral += get_shooter_vel() == 0 ? 0 : err * dt;

        double differential = (err - err_last) / dt;

        double P = err * kP;
        double I = integral * kI;
        double D = differential * kD;

        shooter_control(power * (P + I + D), speed);

        err_last = err;
        old_t = now;
    }
    public void stop_shooter_thread(){
        shooter_thread.shutdown();
        try{
            if(!shooter_thread.awaitTermination(1, TimeUnit.SECONDS)){
                shooter_thread.shutdownNow();
            }
        } catch (InterruptedException e){
            shooter_thread.shutdownNow();
        }
    }
    public void stop_shooting_process(){
        shooting_process.shutdown();
        try{
            if(!shooting_process.awaitTermination(1, TimeUnit.SECONDS)){
                shooting_process.shutdownNow();
            }
        } catch (InterruptedException e){
            shooting_process.shutdownNow();
        }
    }
    public void bw_control(double power){
        bw.setPower(power);
    }
    public void shooter_control(double power, double min_speed){
        if(get_shooter_vel() >= min_speed){
            if(vibro) gamepad2.rumble(200);
            vibro = false;
        } else vibro = true;

        fw.setPower(power);
    }
    public void srv1_control(double pos){
        srv1.setPosition(pos);
    }
    public double get_shooter_vel(){
        return fw.getVelocity();
    }
}