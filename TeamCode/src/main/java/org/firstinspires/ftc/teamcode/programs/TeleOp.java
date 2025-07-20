package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.imu.IMU;
import org.firstinspires.ftc.teamcode.opmodes.superTeleOp;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")

public class TeleOp extends LinearOpMode {
    public void runOpMode() {
        Robot R = new Robot();
        superTeleOp tel = new superTeleOp();
        IMU gyro = new IMU();
        tel.init_classes(hardwareMap, telemetry, gamepad1, gamepad2, this);
        R.init_classes(hardwareMap, telemetry, gamepad1, gamepad2, this);
        R.get_members();
        gyro.calibrate_imu(R.imu);

        waitForStart();
        R.klesh1.setPosition(1);
        while (opModeIsActive()) {
            tel.teleop();
            R.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            R.lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
