package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;
@Autonomous(name="test_Autonomous")
public class test_auto extends LinearOpMode {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild R = new RobotBuild();
        IMU imu = new IMU();
        Wheelbase wheel = new Wheelbase();
        R.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, null, null, wheel, this);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Now is", "%4f", wheel.get_enc_pos());
            telemetry.update();
        }
    }
}