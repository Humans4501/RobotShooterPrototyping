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
	private final int kTopShooterCanID = 32;
	private final int kBottomShooterCanID = 33;
	private final int kFeederCanID = 31;

	//SparkMax controllers
	private final CANSparkMax mShooterTop = new CANSparkMax(this.kTopShooterCanID, MotorType.kBrushless);
	private final RelativeEncoder mShooterTopEnc = this.mShooterTop.getEncoder();
	private final CANSparkMax mShooterBottom = new CANSparkMax(this.kBottomShooterCanID, MotorType.kBrushless);
	private final RelativeEncoder mShooterBtmEnc = this.mShooterBottom.getEncoder();
	private final CANSparkMax mFeedMotor = new CANSparkMax(this.kFeederCanID, MotorType.kBrushless);

	private static final String kMotorSpeedName = "Motor Speed";
	private static final String kFeedSpeedName = "Feed Speed";
	private static final String kSpinupTimeName = "Spinup Time";
	private static final String kFeedTimeName = "Feed Time";
	private static final String kStatusName = "Shooter Status";

	private final Timer mTimer = new Timer();

	private final MutableMeasure<Voltage> mTopAppliedVolts = MutableMeasure.mutable(Units.Volts.of(0.0));
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
				)).angularVelocity(
					this.mTopVelocity.mut_replace(this.mShooterTopEnc.getVelocity(), Units.RadiansPerSecond)
				);
			},
			this
		)
	);

	private final MutableMeasure<Voltage> mBtmAppliedVolts = MutableMeasure.mutable(Units.Volts.of(0.0));
	private final MutableMeasure<Velocity<Angle>> mBtmVelocity = MutableMeasure.mutable(Units.RadiansPerSecond.of(0.0));
	private final SysIdRoutine mSysIdRoutBtm = new SysIdRoutine(
		new SysIdRoutine.Config(),
		new SysIdRoutine.Mechanism(
			(Measure<Voltage> volts) -> {
				this.mShooterTop.setVoltage(volts.in(Units.Volts));
			},
			log -> {
				log.motor("btm-motor")
				.voltage(this.mBtmAppliedVolts.mut_replace(
					this.mShooterTop.get() * RobotController.getBatteryVoltage(), Units.Volts
				)).angularVelocity(
					this.mBtmVelocity.mut_replace(this.mShooterBtmEnc.getVelocity(), Units.RadiansPerSecond)
				);
			},
			this
		)
	);


	public Shooter() {
		// this.mShooterBottom.follow(mShooterTop, true);
		this.mShooterBottom.setInverted(true);
		//returns motor speed and feed speed
		SmartDashboard.setDefaultNumber(kMotorSpeedName, 0);
		SmartDashboard.setDefaultNumber(kFeedSpeedName, 0);
		SmartDashboard.setDefaultNumber(kSpinupTimeName, 2.0);
		SmartDashboard.setDefaultNumber(kFeedTimeName, 1.0);
		SmartDashboard.setDefaultString(kStatusName, "");

		this.setDefaultCommand(this.run(() -> {
			this.mShooterTop.stopMotor();
			this.mFeedMotor.stopMotor();
			this.mTimer.stop();
			SmartDashboard.putString(kStatusName, "Stopped shooting");
		}));

		// Print out expected CAN IDs
		System.out.println(
			"Can IDs:" +
			"\n\tTop shooter motor: " + kTopShooterCanID +
			"\n\tBottom shooter motor: " + kBottomShooterCanID +
			"\n\tFeed motor: " + kFeederCanID
		);
	}

	private Command deferredWaitCommand(DoubleSupplier time) {
		return this.defer(() -> new WaitCommand(time.getAsDouble()));
	}
	
	public Command cmdShoot() {
		final Command shootCmd = new SequentialCommandGroup(
			this.runOnce(() -> {
				//Restarts timer and sets the speed
				this.mShooterTop.set(SmartDashboard.getNumber(kMotorSpeedName, 0));
				this.mTimer.restart();
				SmartDashboard.putString(kStatusName, "Spinning up...");
			}),
			this.deferredWaitCommand(() -> SmartDashboard.getNumber(kSpinupTimeName, 2.0)),
			//Sets speed of feed motor
			this.runOnce(() -> {
				this.mFeedMotor.set(SmartDashboard.getNumber(kFeedSpeedName, 0));
				SmartDashboard.putString(kStatusName, "Feeding");
			}),
			this.deferredWaitCommand(() -> SmartDashboard.getNumber(kFeedTimeName, 1.0)),
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
