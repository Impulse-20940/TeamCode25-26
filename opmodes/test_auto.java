package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;
@Autonomous
public class test_auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild R = new RobotBuild();
        IMU imu = new IMU();
        Cannon cannon = new Cannon();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        R.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, null, null, wheel, this);
        waitForStart();
        R.delay(1500);
        R.turn(-120, 1000);

    }
}