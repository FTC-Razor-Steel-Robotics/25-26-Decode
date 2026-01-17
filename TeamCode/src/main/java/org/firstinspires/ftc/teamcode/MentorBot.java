package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MentorBot extends Robot {
	@Config
	public static class MentorDriveConfig extends DriveConfig {
		public static String frontLeftDriveString = "FL/LO";
		public static String backLeftDriveDriveString = "RL";
		public static String frontRightDriveString = "FR/FO";
		public static String backRightDriveString = "RR";

		public String[] getDriveStrings() {
			return new String[]{
					frontLeftDriveString,
					backLeftDriveDriveString,
					frontRightDriveString,
					backRightDriveString
			};
		}

		public static boolean frontLeftDriveReverse = true;
		public static boolean backLeftDriveDriveReverse = true;
		public static boolean frontRightDriveReverse = false;
		public static boolean backRightDriveReverse = false;

		public boolean[] getDriveReversals() {
			return new boolean[]{
					frontLeftDriveReverse,
					backLeftDriveDriveReverse,
					frontRightDriveReverse,
					backRightDriveReverse
			};
		}
	}

	@Config
	public static class MentorShooterConfig extends ShooterConfig {
		public static String intakeString = "intake";
		public static String shooterString = "shooter";
		public static String shooterDeliverString = "shooterDeliver";
		public static String carouselDeliverString = "carouselDeliver";
		public static String carouselString = "carousel";

		@Override
		public String getShooterString() {
			return shooterString;
		}

		public static double[] shooterSpeeds = {0.8, 0.75, 0.7};
		public static double[] shooterVoltages = {11, 11, 11};

		public static double[] carouselPositions = {0, 0.5, 1};
		public static double[] carouselDeliverPositions = {0, 0.5, 1};
		public static double[] shooterDeliverPositions = {0, 0.5};

		public double[] getShooterSpeeds() {
			return shooterSpeeds;
		}

		public double[] getShooterVoltages() {
			return shooterVoltages;
		}

		public double[] getShooterSpeedsCompensated() {
			int numSpeeds = shooterSpeeds.length;
			double[] compensated = new double[numSpeeds];

			for (int i = 0; i < numSpeeds; i++) {
				compensated[i] = shooterSpeeds[i] * shooterVoltages[i];
			}

			return compensated;
		}
	}

	public enum CarouselPos {
		LEFT,
		CENTER,
		RIGHT;

		//Define functions to make cycling through positions easier
		public CarouselPos next() {
			if (ordinal() == values().length - 1)
				return values()[0];

			return values()[ordinal() + 1];
		}

		public CarouselPos prev() {
			if (ordinal() == 0)
				return values()[values().length - 1];

			return values()[ordinal() - 1];
		}
	}

	public enum CarouselDeliverPos {
		INTAKE,
		CAROUSEL,
		SHOOTER
	}

	private CarouselPos curCarouselPos;
	private CarouselDeliverPos curCarouselDeliverPos;

	private CRServo intake;
	private Servo shooterDeliver;
	private Servo carouselDeliver;
	private Servo carousel;

	public MentorBot(HardwareMap hwMap, Telemetry telem) {
		super(hwMap, telem);

		driveConfig = new MentorDriveConfig();
		shooterConfig = new MentorShooterConfig();

		initDrive();
		initShooter();
	}

	public void initShooter() {
		super.initShooter();

		shooterDeliver = hardwareMap.get(Servo.class, MentorShooterConfig.shooterDeliverString);
		shooterDeliver.setDirection(Servo.Direction.REVERSE);

		intake = hardwareMap.get(CRServo.class, MentorShooterConfig.intakeString);
		intake.setDirection(DcMotorSimple.Direction.REVERSE);

		carouselDeliver = hardwareMap.get(Servo.class, MentorShooterConfig.carouselDeliverString);
		carouselDeliver.setDirection(Servo.Direction.REVERSE);

		carousel = hardwareMap.get(Servo.class, MentorShooterConfig.carouselString);

		moveCarouselDeliver(CarouselDeliverPos.CAROUSEL);
		moveCarousel(CarouselPos.CENTER);
	}

	public void spinIntake(double speed) {
		//Verify that the carousel is in the proper position
		//Skip verification if we are dispensing
		if (curCarouselDeliverPos == CarouselDeliverPos.INTAKE || speed < 0)
			intake.setPower(speed);
		else
			intake.setPower(0);
	}

	public void cycleCarousel(boolean left, boolean right) {
		if (left)
			moveCarousel(curCarouselPos.prev());
		else if (right)
			moveCarousel(curCarouselPos.next());
	}

	private void moveCarousel(CarouselPos pos) {
		if (curCarouselDeliverPos == CarouselDeliverPos.CAROUSEL) {
			carousel.setPosition(MentorShooterConfig.carouselPositions[pos.ordinal()]);
			curCarouselPos = pos;
		}
	}

	public void moveCarouselDeliver(CarouselDeliverPos pos) {
		carouselDeliver.setPosition(MentorShooterConfig.carouselDeliverPositions[pos.ordinal()]);
		curCarouselDeliverPos = pos;
	}
}
