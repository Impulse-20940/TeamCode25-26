package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Cannon;
import org.firstinspires.ftc.teamcode.IMU;
import org.firstinspires.ftc.teamcode.RobotBuild;
import org.firstinspires.ftc.teamcode.Wheelbase;

@Config
@Autonomous(name = "Main_Autonomous")
public class Auto extends LinearOpMode {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    public static double x;
    public static double y;

    public static double x1;
    public static double y1;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBuild r = new RobotBuild();
        Camera cam = new Camera();
        Wheelbase wheel = new Wheelbase();
        IMU imu = new IMU();
        r.init(hardwareMap, telemetry, gamepad1,
                gamepad2, imu, null, cam, wheel, this);

        wheel.reset_encoders();
        waitForStart();

        cam.set_processor();
        while(opModeIsActive()){
            double[] pos = cam.get_position();
            if(pos[0] != 0){
                break;
            }
            telemetry.addData("Detected", "%4f", pos[0]);
            telemetry.update();
        }
        while(opModeIsActive()){
            double[] pos = cam.get_position();
            x = pos[1]-1.8;
            y = pos[2]-10;
            telemetry.addData("Now is", "%1f, %1f, %1f, %1f, %1f", pos[0], x, y,
                                                                    pos[2], pos[7]);
            telemetry.update();

            r.turn(0, 0.007);
            wheel.reset_encoders();
            r.delay(1000);
            r.move_xy(x, x1, y, y1, 0,0.0099);
            wheel.setMPower(0, 0, 0, 0);
            wheel.setZPB();
            break;
        }
        cam.stop_stream();
    }
}
