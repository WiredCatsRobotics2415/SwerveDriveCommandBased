package frc.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class PIDValue implements Cloneable {
    private double kP, kI, kD;

    public PIDValue(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setKP(double kP) {
        this.kP = kP;
    }

    public void setKI(double kI) {
        this.kI = kI;
    }

    public void setKD(double kD) {
        this.kD = kD;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getKP() {
        return kP;
    }

    public double getKI() {
        return kI;
    }

    public double getKD() {
        return kD;
    }

    public void configProfiledPIDController(ProfiledPIDController controller) {
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
    }

    public PIDValue clone() {
        return new PIDValue(kP, kI, kD);
    }
}
