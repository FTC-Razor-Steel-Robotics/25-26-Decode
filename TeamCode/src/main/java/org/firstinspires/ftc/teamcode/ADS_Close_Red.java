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


@Autonomous(name="ADS_Red_close", group="Linear OpMode")
public class ADS_Close_Red extends LinearOpMode {
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
        camera.targetID = camera.RED_ID;

        Pose2d Drive_from_start = new Pose2d(-67, 39, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap,Drive_from_start);

        boolean spinShooter = false;
        camera.update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        camera.update();
        spinShooter = true;
        robot.fireShooterDistance(spinShooter, 55);
        Actions.runBlocking(
                drive.actionBuilder(Drive_from_start)
                        .strafeTo(new Vector2d(-67,9))
                        .build()
        );

        robot.moveGuard(true);
        sleep(400);
        for (int i = 0; i < 2; i++) {
            robot.spinIntake(1);
            sleep(200);
            robot.spinIntake(0);
            sleep(800);
        }
        robot.spinIntake(1);
        sleep(200);
        robot.spinIntake(0);
        robot.moveTrigger(true);
        sleep(1000);
        robot.moveTrigger(false);
        robot.moveGuard(false);
        robot.spinIntake(1);
        spinShooter=false;
        robot.fireShooterDistance(spinShooter, 55);
        Pose2d LP_P1 = new Pose2d(new Vector2d(-67,9),Math.toRadians(270));

        Actions.runBlocking(


                drive.actionBuilder(LP_P1)

                        .strafeToLinearHeading(new Vector2d(-12,15),Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-12,55),Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-12,40),Math.toRadians(90))

                        .build());

        robot.spinIntake(0);
        Pose2d P1_LP = new Pose2d(new Vector2d(-12,40),Math.toRadians(90));
        camera.update();
        spinShooter = true;
        robot.fireShooterDistance(spinShooter, 55);
        Actions.runBlocking(
                drive.actionBuilder(P1_LP)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-23.50, 24.35, Math.toRadians(315)), Math.toRadians(315))
                        .build()
        );
        robot.moveGuard(true);
        sleep(400);
        for (int i = 0; i < 2; i++) {
            robot.spinIntake(1);
            sleep(200);
            robot.spinIntake(0);
            sleep(800);
        }
        robot.spinIntake(1);
        sleep(200);
        robot.spinIntake(0);
        robot.moveTrigger(true);
        sleep(1000);
        robot.moveTrigger(false);
        robot.moveGuard(false);
        robot.spinIntake(1);
        spinShooter=false;
        robot.fireShooterDistance(spinShooter, 55);
        Pose2d LP_P2 = new Pose2d(new Vector2d(-23.50,24.35),Math.toRadians(45));
        Actions.runBlocking(
                drive.actionBuilder(LP_P2)
                        .strafeToLinearHeading(new Vector2d(12,24.35),Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(12,55))
                        .build()

        );
        robot.spinIntake(0);
        Pose2d P2_LP = new Pose2d(new Vector2d(12,45),Math.toRadians(90));
        camera.update();
        spinShooter = true;
        robot.fireShooterDistance(spinShooter, camera.getCameraDistance());
        Actions.runBlocking(
                drive.actionBuilder(P2_LP)
                        .strafeToLinearHeading(new Vector2d(12,35),Math.toRadians(90))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-23.50, 24.35, Math.toRadians(315)), Math.toRadians(315))



                        .build()
        );
        robot.moveGuard(true);
        sleep(400);
        for (int i = 0; i < 2; i++) {
            robot.spinIntake(1);
            sleep(300);
            robot.spinIntake(0);
            sleep(1000);
        }
        robot.spinIntake(1);
        sleep(200);
        robot.spinIntake(0);
        robot.moveTrigger(true);
        sleep(1000);
        robot.moveTrigger(false);
        Pose2d LP_EP = new Pose2d(new Vector2d(-23.50,24.35),Math.toRadians(315));
        Actions.runBlocking(
                drive.actionBuilder(LP_EP)
                        .strafeToLinearHeading(new Vector2d(-10,24.35),Math.toRadians(350))
                        .build()


        );




    }
}


