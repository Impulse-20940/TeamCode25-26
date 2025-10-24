package org.firstinspires.ftc.teamcode.opmodes;
import android.opengl.Matrix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    boolean st90;
    boolean flag;
    double axial;
    double lateral;
    double yaw;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        IMU imu = new IMU();
        //Cannon cannon = new Cannon();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, null, null, wheel, this);
        //cam.set_processor();
        waitForStart();
        while(opModeIsActive()){
            //cam.telemetryAprilTag();
            boolean btn_b = gamepad1.b;
            double rate = 1 - gamepad1.right_trigger;
            if(btn_b){
                if(flag) st90 = !st90;
                flag = false;
            } else flag = true;
            if(st90){
                axial = -gamepad1.left_stick_x*rate;
                lateral = -gamepad1.left_stick_y*rate;
                yaw = imu.get_st_err(-90, 0.012);
            } else {
                axial = gamepad1.left_stick_y*rate;
                lateral = -gamepad1.left_stick_x*rate;
                yaw = -gamepad1.right_stick_x*rate;
            }
            double lfp = axial + lateral + yaw;
            double rfp = axial - lateral - yaw;
            double lbp = axial - lateral + yaw;
            double rbp = axial + lateral - yaw;

            wheel.setMPower(rbp, rfp, lfp, lbp);
            wheel.setZPB();
        }
        //cam.stop_stream();
    }
}
