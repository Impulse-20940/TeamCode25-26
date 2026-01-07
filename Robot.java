package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Robot {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public LinearOpMode L;
    public ElapsedTime runtime;
    public DcMotorEx rf;
    public DcMotor rb;
    public DcMotor lf;
    public DcMotor lb;
    public DcMotor c2;
    public DcMotorEx fw;
    public BNO055IMU imu;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public Servo srv1;
    IMU Imu;
    Wheelbase wb;
    Cannon cnn;
    Camera cam;
    public void delay(long millis){
        try{
            Thread.sleep(millis);
        } catch (InterruptedException ex){
            Thread.currentThread().interrupt();
        }
    }
}
