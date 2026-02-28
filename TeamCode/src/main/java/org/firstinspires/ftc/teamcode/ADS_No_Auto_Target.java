package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configs.CompShooterConfig;
import org.firstinspires.ftc.teamcode.configs.PIDController;

import java.util.List;


@Autonomous(name="ADS_NAT_Blue_Close", group="Linear OpMode")
public class ADS_No_Auto_Target extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    CompBot robot;
    Limelight3A limelight;
    PIDController pid = new PIDController(CompShooterConfig.kP, CompShooterConfig.kI, CompShooterConfig.kD);
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new CompBot(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();


        Pose2d Drive_from_start = new Pose2d(-67, -15, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap,Drive_from_start);

        boolean spinShooter = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        spinShooter = true;
        robot.fireShooter(spinShooter,1);
        sleep(3000);
        robot.moveGuard(true);
        sleep(250);
        for (int i = 0; i < 3; i++) {
            robot.spinIntake(1);
            sleep(200);
            robot.spinIntake(0);
            sleep(550);
        }
        robot.moveGuard(false);
        robot.spinIntake(1);
        spinShooter=false;
        robot.fireShooter(spinShooter,1);

        Actions.runBlocking(


                drive.actionBuilder(Drive_from_start)

                        .strafeToLinearHeading(new Vector2d(-12,-15),Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-12,-47),Math.toRadians(270))
                        .build());

        robot.spinIntake(0);
        Pose2d P1_LP = new Pose2d(new Vector2d(-12,-45),Math.toRadians(270));
        spinShooter=true;
        robot.fireShooter(spinShooter,1);
        Actions.runBlocking(
                drive.actionBuilder(P1_LP)
                        .splineToLinearHeading(new Pose2d(-23.50, -24.35, Math.toRadians(45.00)), Math.toRadians(45.00))
                        .build()
        );
        robot.moveGuard(true);
        sleep(250);
        for (int i = 0; i < 3; i++) {
            robot.spinIntake(1);
            sleep(200);
            robot.spinIntake(0);
            sleep(550);
        }
        robot.moveGuard(false);
        robot.spinIntake(1);
        spinShooter=false;
        robot.fireShooter(spinShooter,1);
        Pose2d LP_P2 = new Pose2d(new Vector2d(-23.50,-24.35),Math.toRadians(45));
        Actions.runBlocking(
                drive.actionBuilder(LP_P2)
                        .strafeToLinearHeading(new Vector2d(12,-24.35),Math.toRadians(270))
                        .strafeToConstantHeading(new Vector2d(12,-45))
                        .build()

        );
        robot.spinIntake(0);
        Pose2d P2_LP = new Pose2d(new Vector2d(12,-45),Math.toRadians(270));
        spinShooter=true;
        robot.fireShooter(spinShooter,1);
        Actions.runBlocking(
                drive.actionBuilder(P2_LP)
                        .splineToLinearHeading(new Pose2d(-23.50, -24.35, Math.toRadians(45.00)), Math.toRadians(45.00))
                        .build()
        );
        robot.moveGuard(true);
        for (int i = 0; i < 3; i++) {
            robot.spinIntake(1);
            sleep(200);
            robot.spinIntake(0);
            sleep(550);
        }



    }
}


