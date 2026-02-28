package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.CompShooterConfig;
import org.firstinspires.ftc.teamcode.configs.PIDController;


@Autonomous(name="ADS_Blue_close", group="Linear OpMode")
public class ADS_Close_Blue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    CompBot robot;
    Limelight3A limelight;
    PIDController pid = new PIDController(CompShooterConfig.kP, CompShooterConfig.kI, CompShooterConfig.kD);
    Camera camera;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        robot = new CompBot(hardwareMap, telemetry);
        camera = new Camera(hardwareMap, telemetry, runtime);


        Pose2d Drive_from_start = new Pose2d(-67, -15, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap,Drive_from_start);

        boolean spinShooter = false;
        camera.update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        camera.update();
        spinShooter = true;
        robot.fireShooterDistance(spinShooter, 45);
        sleep(2500);
        robot.moveGuard(true);
        sleep(400);
        for (int i = 0; i < 3; i++) {
            robot.spinIntake(1);
            sleep(200);
            robot.spinIntake(0);
            sleep(800);
        }
        sleep(400);
        robot.moveGuard(false);
        robot.spinIntake(1);
        spinShooter=false;
        robot.fireShooter(spinShooter,1);

        Actions.runBlocking(


                drive.actionBuilder(Drive_from_start)

                        .strafeToLinearHeading(new Vector2d(-12,-15),Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-12,-55),Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-12,-40),Math.toRadians(270))

                        .build());

        robot.spinIntake(0);
        Pose2d P1_LP = new Pose2d(new Vector2d(-12,-45),Math.toRadians(270));
        camera.update();
        spinShooter = true;
        robot.fireShooterDistance(spinShooter, camera.getCameraDistance());
        Actions.runBlocking(
                drive.actionBuilder(P1_LP)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-23.50, -24.35, Math.toRadians(45.00)), Math.toRadians(45.00))
                        .setReversed(false)
                        .turnTo(Math.toRadians(45))
                        .build()
        );
        robot.moveGuard(true);
        sleep(400);
        for (int i = 0; i < 3; i++) {
            robot.spinIntake(1);
            sleep(200);
            robot.spinIntake(0);
            sleep(800);
        }
        sleep(400);
        robot.moveGuard(false);
        robot.spinIntake(1);
        spinShooter=false;
        robot.fireShooter(spinShooter,1);
        Pose2d LP_P2 = new Pose2d(new Vector2d(-23.50,-24.35),Math.toRadians(45));
        Actions.runBlocking(
                drive.actionBuilder(LP_P2)
                        .strafeToLinearHeading(new Vector2d(12,-24.35),Math.toRadians(270))
                        .strafeToConstantHeading(new Vector2d(12,-55))
                        .build()

        );
        robot.spinIntake(0);
        Pose2d P2_LP = new Pose2d(new Vector2d(12,-45),Math.toRadians(270));
        camera.update();
        spinShooter = true;
        robot.fireShooterDistance(spinShooter, camera.getCameraDistance());
        Actions.runBlocking(
                drive.actionBuilder(P2_LP)
                        .strafeToLinearHeading(new Vector2d(12,-35),Math.toRadians(270))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-23.50, -24.35, Math.toRadians(45.00)), Math.toRadians(45.00))
                        .setReversed(false)
                        .turnTo(Math.toRadians(45))

                        .build()
        );
        robot.moveGuard(true);
        sleep(400);
        for (int i = 0; i < 3; i++) {
            robot.spinIntake(1);
            sleep(200);
            robot.spinIntake(0);
            sleep(800);
        }
        Pose2d LP_EP = new Pose2d(new Vector2d(-23.50,-24.35),Math.toRadians(10));
        Actions.runBlocking(
                drive.actionBuilder(LP_EP)
                        .strafeToLinearHeading(new Vector2d(-15,-24.35),Math.toRadians(10))
                        .build()


        );




    }
}


