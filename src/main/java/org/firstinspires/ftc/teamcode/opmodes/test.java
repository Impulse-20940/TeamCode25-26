package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="test_TeleOp")
public class test extends LinearOpMode {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        IMU imu = new IMU();
        //Cannon cannon = new Cannon();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, null, cam, wheel, this);
        cam.set_processor();
        waitForStart();
        while (opModeIsActive()) {
            double[] pos = cam.get_position();
            telemetry.addData("Now is ", "%4f, %4f, %4f, %4f", pos[1], pos[2], pos[3], pos[6]);
            telemetry.update();
        }
        cam.stop_stream();

    }

}
