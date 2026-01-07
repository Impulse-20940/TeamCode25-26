package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wheelbase {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;
    ElapsedTime runtime;
    DcMotorEx rf;
    DcMotor rb;
    DcMotor lf;
    DcMotor lb;

    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                             DcMotorEx RF, DcMotor RB, DcMotor LF, DcMotor LB,
                             LinearOpMode L){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.rf = RF;
        this.rb = RB;
        this.lf = LF;
        this.lb = LB;
        this.L = L;
        rb = hardwareMap.get(DcMotor.class, "rf");
        rf = hardwareMap.get(DcMotorEx.class, "rb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void reset_encoders(){
        rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void setMPower(double RB, double RF, double LF, double LB){
        rf.setPower(RF);
        lf.setPower(LF);
        rb.setPower(RB);
        lb.setPower(LB);
    }
    public void setZPB(){
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public double get_enc_pos(){
        return rf.getCurrentPosition();
    }
    public void telemetry_ports(){
        telemetry.addData("RF is in", rf.getPortNumber());
        telemetry.addData("RB is in", rb.getPortNumber());
        telemetry.addData("LF is in", lf.getPortNumber());
        telemetry.addData("LB is in", lb.getPortNumber());
        telemetry.update();
    }
    public void telemetry_power(){
        telemetry.addData("RF power is", rf.getPower());
        telemetry.addData("RB power is", rb.getPower());
        telemetry.addData("LF power is", lf.getPower());
        telemetry.addData("LB power is", lb.getPower());
        telemetry.update();
    }
}

