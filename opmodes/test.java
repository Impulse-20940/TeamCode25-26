package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;

public class test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        //IMU imu = new IMU();
        //Cannon cannon = new Cannon();
        //Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, null, null, null, wheel, this);
        //cam.set_processor();
        waitForStart();
    }
}
