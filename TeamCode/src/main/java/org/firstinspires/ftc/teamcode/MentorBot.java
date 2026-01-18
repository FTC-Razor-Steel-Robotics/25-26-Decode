package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
			return new String[] {
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
			return new boolean[] {
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

		public static long intakeTime = 2000;

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

	public static class MentorRRConfig extends RRConfig {
		public static String rightOdomString = "RO";
		public static String leftOdomString = MentorDriveConfig.frontLeftDriveString;
		public static String frontOdomString = MentorDriveConfig.frontRightDriveString;

		public String[] getOdomStrings() {
			return new String[] {
					rightOdomString,
					leftOdomString,
					frontOdomString
			};
		}

		public RevHubOrientationOnRobot.LogoFacingDirection getLogoDirection() {
			return RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
		}

		public RevHubOrientationOnRobot.UsbFacingDirection getUSBDirection() {
			return RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
		}
	}

	@Config
	public static class MentorLiftConfig {
		public static String frontLiftString = "frontLift";
		public static String rearLiftString = "rearLift";

		public static boolean frontLiftReverse = true;
		public static boolean rearLiftReverse = true;

		public static double liftSpeed = 0.3;
		public static int liftMax = 500;
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

	private DcMotor frontLift;
	private DcMotor rearLift;

	private CRServo intake;
	private Servo shooterDeliver;
	private Servo carouselDeliver;
	private Servo carousel;

	//Boolean to keep track of our threads
	private static volatile boolean intakeBusy = false;

	public static DriveConfig driveConfig = new MentorDriveConfig();
	public static ShooterConfig shooterConfig = new MentorShooterConfig();
	public static RRConfig rrConfig = new MentorRRConfig();

	public MentorBot(HardwareMap hwMap, Telemetry telem) {
		super(hwMap, telem);

		driveConfig = new MentorDriveConfig();
		shooterConfig = new MentorShooterConfig();

		initDrive();
		initShooter();
		initLift();
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

		//Allow time for the carousel deliver to get out of the way
		robotSleep(250);

		moveCarousel(CarouselPos.CENTER);
	}

	public void initLift() {
		frontLift = hardwareMap.get(DcMotor.class, MentorLiftConfig.frontLiftString);
		rearLift = hardwareMap.get(DcMotor.class, MentorLiftConfig.rearLiftString);

		frontLift.setDirection(MentorLiftConfig.frontLiftReverse ?
								DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
		rearLift.setDirection(MentorLiftConfig.rearLiftReverse ?
				DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

		frontLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		frontLift.setTargetPosition(0);
		rearLift.setTargetPosition(0);

		frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		frontLift.setPower(MentorLiftConfig.liftSpeed);
		rearLift.setPower(MentorLiftConfig.liftSpeed);
	}

	//Shorthand function to intake into the current carousel selection
	public void autoIntake() {
		autoIntake(curCarouselPos);
	}

	//TODO: Utilize a sensor to determine when a ball has arrived in the carousel deliver
	public void autoIntake(CarouselPos pos) {
		Thread thread = new Thread() {
			@Override
			public void run() {
				intakeBusy = true;

				//Rotate the carousel to select the right position
				//Check if we can save time
				if (pos != curCarouselPos) {
					//Ensure the carousel deliver won't get in the way
					//Check if we can save time
					if (curCarouselDeliverPos != CarouselDeliverPos.CAROUSEL) {
						moveCarouselDeliver(CarouselDeliverPos.CAROUSEL);

						robotSleep(250);
					}

					moveCarousel(pos);

					//TODO: Add extra time if we are going from LEFT to RIGHT or vice-versa
					// We need 500 ms for the worst case
					// Maybe even use different values for all the different cases
					robotSleep(500);
				}

				//Intake the ball
				moveCarouselDeliver(CarouselDeliverPos.INTAKE);
				//Give the carousel deliver just a bit of time to come down first
				robotSleep(250);
				spinIntake(1);

				robotSleep(MentorShooterConfig.intakeTime);

				//Reset the carousel deliver position
				moveCarouselDeliver(CarouselDeliverPos.CAROUSEL);

				//Assist the carousel deliver with the ball before we stop the intake
				robotSleep(250);

				spinIntake(0);

				intakeBusy = false;
			}
		};

		//Make sure we don't create duplicate threads trying to do the same thing
		if (!intakeBusy)
			thread.start();
	}

	//Shorthand function to dispense the current carousel selection
	public void autoDispense() {
		autoDispense(curCarouselPos);
	}

	public void autoDispense(CarouselPos pos) {
		Thread thread = new Thread() {
			@Override
			public void run() {
				intakeBusy = true;

				//Rotate the carousel to select the right position
				//Check if we can save time
				if (pos != curCarouselPos) {
					//Ensure the carousel deliver won't get in the way
					//Check if we can save time
					if (curCarouselDeliverPos != CarouselDeliverPos.CAROUSEL) {
						moveCarouselDeliver(CarouselDeliverPos.CAROUSEL);

						robotSleep(250);
					}

					moveCarousel(pos);

					//TODO: Add extra time if we are going from LEFT to RIGHT or vice-versa
					// We need 500 ms for the worst case
					// Maybe even use different values for all the different cases
					robotSleep(500);
				}

				//Dispense the ball
				moveCarouselDeliver(CarouselDeliverPos.INTAKE);
				spinIntake(-1);

				robotSleep(MentorShooterConfig.intakeTime);

				//Reset the carousel deliver position
				moveCarouselDeliver(CarouselDeliverPos.CAROUSEL);
				spinIntake(0);

				intakeBusy = false;
			}
		};

		//Make sure we don't create duplicate threads trying to do the same thing
		if (!intakeBusy)
			thread.start();
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

	public void moveCarousel(CarouselPos pos) {
		if (curCarouselDeliverPos == CarouselDeliverPos.CAROUSEL) {
			carousel.setPosition(MentorShooterConfig.carouselPositions[pos.ordinal()]);
			curCarouselPos = pos;
		}
	}

	public void moveCarouselDeliver(CarouselDeliverPos pos) {
		carouselDeliver.setPosition(MentorShooterConfig.carouselDeliverPositions[pos.ordinal()]);
		curCarouselDeliverPos = pos;
	}

	public void moveLift(boolean up, boolean down) {
		if (up) {
			frontLift.setTargetPosition(MentorLiftConfig.liftMax);
			rearLift.setTargetPosition(MentorLiftConfig.liftMax);
		} else if (down) {
			frontLift.setTargetPosition(0);
			rearLift.setTargetPosition(0);
		}

		telemetry.addData("Front Lift Target", frontLift.getTargetPosition());
		telemetry.addData("Front Lift Pos", frontLift.getCurrentPosition());
		telemetry.addData("Rear Lift Target", rearLift.getTargetPosition());
		telemetry.addData("Rear Lift Pos", rearLift.getCurrentPosition());
	}
}
