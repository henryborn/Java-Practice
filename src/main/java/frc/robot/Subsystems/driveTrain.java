package frc.robot.Subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;




public class driveTrain {
    private static final double kwheelradius = .0508;
    private static final int kEncoderRes = 4096;
    public static final int kMaxAngularSpeed = 10;
    public static final int kMaxDriveSpeed = 3;

    private final SparkMax fLMotor = new SparkMax(1, SparkMax.MotorType.kBrushless);
    private final SparkMax fRMotor = new SparkMax(2, SparkMax.MotorType.kBrushless);
    private SparkMax bLMotorfollower = new SparkMax(3, SparkMax.MotorType.kBrushless);
    private SparkMax bRMotorfollower = new SparkMax(4, SparkMax.MotorType.kBrushless);
    public AbsoluteEncoder fLAbsEnc = fLMotor.getAbsoluteEncoder();
    public AbsoluteEncoder fRAbsEnc = fRMotor.getAbsoluteEncoder();
    public Pigeon2 gyro = new Pigeon2(5);
    private final PIDController leftPID = new PIDController(1, 0, 0);
    private final PIDController rightPID = new PIDController(1, 0, 0);
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.69);
    private DifferentialDriveOdometry odometry;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, .5, .1);

    private double leftEncoderOffset;
    private double rightEncoderOffset;

    public driveTrain() {
        gyro.reset();
        leftEncoderOffset = fLAbsEnc.getPosition();
        rightEncoderOffset = fRAbsEnc.getPosition();
        

        odometry =
        new DifferentialDriveOdometry(
            gyro.getRotation2d(), getLeftEncoderPositionMeters(), getRightEncoderPositionMeters());

    }
    public double getLeftEncoderPositionMeters() {
        return (fLAbsEnc.getPosition() - leftEncoderOffset) * (2 * Math.PI * kwheelradius / kEncoderRes);
    }
    
    public double getRightEncoderPositionMeters() {
        return (fRAbsEnc.getPosition() - rightEncoderOffset) * (2 * Math.PI * kwheelradius / kEncoderRes);
    }
    public void setspeeds(DifferentialDriveWheelSpeeds speeds) {
        double leftfeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        double rightfeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

        double leftOutput = leftPID.calculate(getLeftEncoderPositionMeters(), speeds.leftMetersPerSecond);
        double rightOutput = rightPID.calculate(getRightEncoderPositionMeters(), speeds.rightMetersPerSecond);
        fLMotor.setVoltage(leftOutput + leftfeedforward);
        bLMotorfollower.setVoltage(leftOutput + leftfeedforward);
        fRMotor.setVoltage(rightOutput + rightfeedforward);
        bRMotorfollower.setVoltage(rightOutput + rightfeedforward);
    }
    public void drive(double xSpeed, double zRotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, 0, zRotation, gyro.getRotation2d());
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        setspeeds(wheelSpeeds);
    }
      public void resetEncoders() {
        leftEncoderOffset = fLAbsEnc.getPosition();
        rightEncoderOffset = fRAbsEnc.getPosition();
      }
    public void updateOdometry() {
        odometry.update(
            gyro.getRotation2d(), getLeftEncoderPositionMeters(), getRightEncoderPositionMeters());
      }
}
