package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;
import frc.utils.RobotPreferences;

public class OIs {
    public static abstract class OI {
        //PROPERTIES
        public double DEADBAND;

        public Map<String, JoystickButton> binds;

        //JOYSTICKS
        public abstract double getX();

        public abstract double getY();

        public abstract double getRotation();

        //UTILS
        public abstract void setInputPrefrences();
    }
    
    public static class GulikitController extends OI {
        XboxController controller;

        private boolean isCurve;
        private int curve;
        private double slewRate;

        private SlewRateLimiter xLimiter;
        private SlewRateLimiter yLimiter;
        private SlewRateLimiter rLimiter;

        public void setInputPrefrences() {
            if (RobotPreferences.getInputFilter()) {
                //Curve
                isCurve = true;
                curve = RobotPreferences.getCurvePower();
                if (curve < 1) curve = 1; //Note: must include because fractional/negative powers will yield uncontrolable results
                Logger.log(LogLevel.INFO, "OI: Using Curve at " + curve + " power.");
            } else {
                //Slew
                isCurve = false;
                slewRate = RobotPreferences.getSlewRateLimit();
                if (slewRate < 0) slewRate = 1; //Note: must include because negative rates will yield uncontrolable results
                xLimiter = new SlewRateLimiter(slewRate);
                yLimiter = new SlewRateLimiter(slewRate);
                rLimiter = new SlewRateLimiter(slewRate);
                Logger.log(LogLevel.INFO, "OI: Using Slew at " + slewRate + " rate.");
            }
        }

        public GulikitController() {
            controller = new XboxController(0);
            binds = Map.of(
                "navX Reset", new JoystickButton(controller, 7) //Minus
            );
        }

        public final double DEADBAND = 0.05;

        public double getX() {
            if (isCurve) {
                return Math.pow(MathUtil.applyDeadband(controller.getRawAxis(0), DEADBAND), curve);
            } else {
                return xLimiter.calculate(MathUtil.applyDeadband(controller.getRawAxis(0), DEADBAND));
            }
        }

        public double getY() { 
            if (isCurve) {
                return Math.pow(MathUtil.applyDeadband(controller.getRawAxis(1), DEADBAND), curve);
            } else {
                return yLimiter.calculate(MathUtil.applyDeadband(controller.getRawAxis(1), DEADBAND));
            }
        }

        public double getRotation() { 
            if (isCurve) {
                return -Math.pow(MathUtil.applyDeadband(controller.getRawAxis(4), DEADBAND), curve);
            } else {
                return -rLimiter.calculate(MathUtil.applyDeadband(controller.getRawAxis(4), DEADBAND));
            }
        }
    }
}
