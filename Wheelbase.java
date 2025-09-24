package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wheelbase {
    IMU gyro;
    Cannon cannon;
    Camera camera;
    Wheelbase wb;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;
    ElapsedTime runtime;
    DcMotor rf;
    DcMotor rb;
    DcMotor lf;
    DcMotor lb;
    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                             DcMotor RF, DcMotor RB, DcMotor LF, DcMotor LB,
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
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
    }
    public void stable(double stable, long time, double kt){
        runtime.reset();
        while(L.opModeIsActive() && runtime.seconds() < time){
            double getangle = stable-gyro.getTurnAngle();
            double axial = 0;
            double lateral = getangle*kt;
            double yaw = 0;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            lf.setPower(leftFrontPower);
            rf.setPower(rightFrontPower);
            lb.setPower(-leftBackPower);
            rb.setPower(-rightBackPower);
        }
        setMPower(0, 0, 0, 0);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setMPower(double RB, double RF, double LF, double LB){
        rf.setPower(RF);
        lf.setPower(LF);
        rb.setPower(RB);
        lb.setPower(LB);
    }
}
