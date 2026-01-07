package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;
@Config
@Autonomous(name="test_Autonomous")
public class test_auto extends LinearOpMode {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    @Override
    public void runOpMode() {
        RobotBuild R = new RobotBuild();
        IMU imu = new IMU();
        Wheelbase wheel = new Wheelbase();
        Camera cam = new Camera();
        //Cannon cannon = new Cannon();
        R.init(hardwareMap, telemetry, gamepad1,
                gamepad2, null, null, cam, null, this);
        waitForStart();
        cam.set_processor();
        while(opModeIsActive()){
            cam.telemetryAprilTag();
        }
        cam.stop_stream();
    }
}