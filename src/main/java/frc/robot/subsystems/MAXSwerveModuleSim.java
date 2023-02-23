package frc.robot.subsystems;

public class MAXSwerveModuleSim extends MAXSwerveModule{
    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     *
     * @param drivingCANId
     * @param turningCANId
     * @param chassisAngularOffset
     */
    public MAXSwerveModuleSim(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        super(drivingCANId, turningCANId, chassisAngularOffset);
    }
}
