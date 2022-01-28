
package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private CANSparkMax hoodSpark;
  private RelativeEncoder hoodEncoder;
  private SparkMaxLimitSwitch beamBreak;


  public HoodSubsystem() {
    hoodSpark = new CANSparkMax(HoodConstants.HOOD_SPARK, MotorType.kBrushless);  
    hoodEncoder = hoodSpark.getEncoder();
    hoodSpark.setInverted(true);
    beamBreak = hoodSpark.getReverseLimitSwitch(Type.kNormallyClosed);
    hoodSpark.setSoftLimit(SoftLimitDirection.kForward, (float)hoodEncoderFromAngle(HoodConstants.MAX_HOOD_ANGLE));
    hoodSpark.setSoftLimit(SoftLimitDirection.kReverse, (float)hoodEncoderFromAngle(0));

  }

  @Override
  public void periodic() {
  //  System.out.println("hood enc value: " + hoodSpark.getEncoder());
  }

  public void moveHood(double power) {
    hoodSpark.set(power);
  }

  public void stopHood() {
    hoodSpark.set(0);
  }

  public double hoodEncoderFromAngle(double angle){
    return HoodConstants.COUNTS_PER_DEGREE * angle;
  }

  public double hoodAngleFromEncoder(double encoderValue){
    return encoderValue / HoodConstants.COUNTS_PER_DEGREE;
  }

  public void resetEncoder(){
    hoodEncoder.setPosition(0);
}

  public double getEncoder(){
    return hoodEncoder.getPosition();
  }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        

  public boolean getBeamBreak(){
    return beamBreak.isPressed();
  }

   public double calcHoodAngle(double distance) {
     //y = mx+b values based on hood testing
     return ((.0867898* distance) + 12.4589);
   }

}
