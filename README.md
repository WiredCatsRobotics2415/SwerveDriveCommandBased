Swerve Drive command based

using standard 2415 hw:
 - Falcon 500 with talonFXs built in for each motor
 - US MA3 absolute ANALOG (not pwm) encoders as external encoders (basically just potentiometers)

Ideally uses external encoders for PID control

Based off of:
 - 50%: https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot
 - 20%: https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand
 - 30%: https://github.com/WiredCatsRobotics2415/WiredCats2023

At the time this was initially commited, few things to fix:
 - The 2 WPILIB examples both use radians as input to the ProfiledPIDController. Not sure it would work to change everything to use -180,180 for the external encoders which use the AbsoluteAnalogEncoder class which returns between 0,360 but worth a shot
 - Also not really sure how to use Sysid to tune yet so still need to figure that out
 - Finally, we've never used navX.getRotation2d() for field oriented drive, so we'll need to see how that works.
