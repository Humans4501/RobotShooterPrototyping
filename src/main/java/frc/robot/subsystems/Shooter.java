package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Shooter extends SubsystemBase {
	private static final int kTopShooterCanID = 32;
	private static final int kBottomShooterCanID = 33;
	private static final int kFeederCanID = 31;

	//SparkMax controllers
	private final CANSparkMax mShooterTop = new CANSparkMax(Shooter.kTopShooterCanID, MotorType.kBrushless);
	private final RelativeEncoder mShooterTopEnc = this.mShooterTop.getEncoder();
	private final CANSparkMax mShooterBottom = new CANSparkMax(Shooter.kBottomShooterCanID, MotorType.kBrushless);
	private final RelativeEncoder mShooterBtmEnc = this.mShooterBottom.getEncoder();
	private final CANSparkMax mFeedMotor = new CANSparkMax(Shooter.kFeederCanID, MotorType.kBrushless);

	private static final String kMotorSpeedName = "Motor Speed";
	private static final String kFeedSpeedName = "Feed Speed";
	private static final String kSpinupTimeName = "Spinup Time";
	private static final String kStatusName = "Shooter Status";

	private final Timer mTimer = new Timer();

	private final MutableMeasure<Voltage> mTopAppliedVolts = MutableMeasure.mutable(Units.Volts.of(0.0));
	private final MutableMeasure<Angle> mTopPos = MutableMeasure.mutable(Units.Radians.of(0.0));
	private final MutableMeasure<Velocity<Angle>> mTopVelocity = MutableMeasure.mutable(Units.RadiansPerSecond.of(0.0));
	private final SysIdRoutine mSysIdRoutTop = new SysIdRoutine(
		new SysIdRoutine.Config(),
		new SysIdRoutine.Mechanism(
			(Measure<Voltage> volts) -> {
				this.mShooterTop.setVoltage(volts.in(Units.Volts));
			},
			log -> {
				log.motor("top-motor")
				.voltage(this.mTopAppliedVolts.mut_replace(
					this.mShooterTop.get() * RobotController.getBatteryVoltage(), Units.Volts
				))
				.angularPosition(this.mTopPos.mut_replace(
					this.mShooterTopEnc.getPosition(), Units.Radians
				))
				.angularVelocity(this.mTopVelocity.mut_replace(
					this.mShooterTopEnc.getVelocity(), Units.RadiansPerSecond
				));
			},
			new SubsystemBase() {}
		)
	);

	private final MutableMeasure<Voltage> mBtmAppliedVolts = MutableMeasure.mutable(Units.Volts.of(0.0));
	private final MutableMeasure<Angle> mBtmPos = MutableMeasure.mutable(Units.Radians.of(0.0));
	private final MutableMeasure<Velocity<Angle>> mBtmVelocity = MutableMeasure.mutable(Units.RadiansPerSecond.of(0.0));
	private final SysIdRoutine mSysIdRoutBtm = new SysIdRoutine(
		new SysIdRoutine.Config(),
		new SysIdRoutine.Mechanism(
			(Measure<Voltage> volts) -> {
				this.mShooterBottom.setVoltage(volts.in(Units.Volts));
			},
			log -> {
				log.motor("btm-motor")
				.voltage(this.mBtmAppliedVolts.mut_replace(
					this.mShooterBottom.get() * RobotController.getBatteryVoltage(), Units.Volts
				))
				.angularPosition(this.mBtmPos.mut_replace(
					this.mShooterBtmEnc.getPosition(), Units.Radians
				))
				.angularVelocity(
					this.mBtmVelocity.mut_replace(this.mShooterBtmEnc.getVelocity(), Units.RadiansPerSecond)
				);
			},
			new SubsystemBase() {}
		)
	);

	public Shooter() {
		// this.mShooterBottom.follow(mShooterTop, true);
		this.mShooterBottom.setInverted(true);

		this.mShooterTopEnc.setPositionConversionFactor(2.0 * Math.PI);
		this.mShooterBtmEnc.setPositionConversionFactor((2.0 * Math.PI) / 60.0);

		//returns motor speed and feed speed
		SmartDashboard.setDefaultNumber(Shooter.kMotorSpeedName, 0);
		SmartDashboard.setDefaultNumber(Shooter.kFeedSpeedName, 0);
		SmartDashboard.setDefaultNumber(Shooter.kSpinupTimeName, 2.0);
		SmartDashboard.setDefaultString(Shooter.kStatusName, "");

		this.setDefaultCommand(this.run(() -> {
			this.mShooterTop.stopMotor();
			this.mShooterBottom.stopMotor();
			this.mFeedMotor.stopMotor();
			this.mTimer.stop();
			SmartDashboard.putString(Shooter.kStatusName, "Stopped shooting");
		}));

		// Print out expected CAN IDs
		System.out.println(
			"Can IDs:" +
			"\n\tTop shooter motor: " + Shooter.kTopShooterCanID +
			"\n\tBottom shooter motor: " + Shooter.kBottomShooterCanID +
			"\n\tFeed motor: " + Shooter.kFeederCanID
		);
	}

	private Command deferredWaitCommand(DoubleSupplier time) {
		return this.defer(() -> new WaitCommand(time.getAsDouble()));
	}

	public Command cmdShoot() {
		final Command shootCmd = new SequentialCommandGroup(
			this.runOnce(() -> {
				//Restarts timer and sets the speed
				final double speed = SmartDashboard.getNumber(Shooter.kMotorSpeedName, 0);
				this.mShooterTop.set(speed);
				this.mShooterBottom.set(speed);
				this.mTimer.restart();
				SmartDashboard.putString(Shooter.kStatusName, "Spinning up...");
			}),
			this.deferredWaitCommand(() -> SmartDashboard.getNumber(Shooter.kSpinupTimeName, 2.0)),
			//Sets speed of feed motor
			this.runOnce(() -> {
				this.mFeedMotor.set(SmartDashboard.getNumber(Shooter.kFeedSpeedName, 0));
				SmartDashboard.putString(Shooter.kStatusName, "Feeding");
			}),
			new RepeatCommand(new InstantCommand())
		);

		shootCmd.addRequirements(this);

		return shootCmd;
	}

	public Command sysidTopQuasi() {
		return this.mSysIdRoutTop.quasistatic(Direction.kForward);
	}

	public Command sysidBtmQuasi() {
		return this.mSysIdRoutBtm.quasistatic(Direction.kForward);
	}

	public Command sysidTopDynamic() {
		return this.mSysIdRoutTop.dynamic(Direction.kForward);
	}

	public Command sysidBtmDynamic() {
		return this.mSysIdRoutBtm.dynamic(Direction.kForward);
	}
}
