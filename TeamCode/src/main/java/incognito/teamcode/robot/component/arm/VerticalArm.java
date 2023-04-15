package incognito.teamcode.robot.component.arm;

import static incognito.teamcode.config.WorldSlideConstants.S_RUN_TO_POSITION_POWER;
import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_TO_INTAKE_TIME;
import static incognito.teamcode.config.WorldSlideConstants.VS_HINGE_TO_INTAKE_TIME_LOW;
import static incognito.teamcode.config.WorldSlideConstants.VS_OFFSET_FACTOR;
import static incognito.teamcode.robot.component.arm.HorizontalArm.Position.MANUAL;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import incognito.teamcode.config.WorldSlideConstants;
import incognito.teamcode.robot.component.servoImplementations.Claw;
import incognito.teamcode.robot.component.servoImplementations.VerticalHinge;
import incognito.teamcode.robot.component.servoImplementations.Lever;
import incognito.teamcode.robot.component.slide.VerticalSlide;

public class VerticalArm extends Arm {

    public enum Position {
        INTAKE, LOW, MEDIUM, HIGH, INTAKE_LIMIT;

        public double getWaitTime(Position currentPosition) {
            return WorldSlideConstants.VS_TIME_TO.getTime(this.name(), currentPosition.name());
        }
    }

    public VerticalSlide slide;
    public Lever lever;
    public VerticalHinge hinge;
    public Claw claw;
    private Position currentPosition;
    private Position lastPosition;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public VerticalArm(VerticalSlide slide, Lever lever, VerticalHinge hinge, Claw claw) {
        this.slide = slide;
        slide.setOffsetFactor(VS_OFFSET_FACTOR);
        this.lever = lever;
        this.hinge = hinge;
        this.claw = claw;
        goToPosition(VerticalArm.Position.INTAKE);
    }

    public void goToPosition(Position position) {
        if (position == currentPosition) return;
        if (currentPosition == null) {
            currentPosition = position;
        }
        timer.reset();
        lastPosition = getPosition();
        currentPosition = position;
        // THIS MIGHT BE SKETCHY BUT IT COULD WORK
        slide.enable();
        switch (position) {
            case INTAKE_LIMIT:
                slide.setPower(-1);
                lever.goToSetPosition(Lever.VerticalLeverPosition.INTAKE);
                if (lastPosition == Position.LOW) {
                    hinge.goToSetPosition(VerticalHinge.Position.INTAKE, false, VS_HINGE_TO_INTAKE_TIME_LOW);
                } else {
                    hinge.goToSetPosition(VerticalHinge.Position.INTAKE, lastPosition == Position.INTAKE, VS_HINGE_TO_INTAKE_TIME);
                }
                openClaw();
                break;
            case INTAKE:
                slide.goToSetPosition(VerticalSlide.Position.INTAKE);
                lever.goToSetPosition(Lever.VerticalLeverPosition.INTAKE);
                if (lastPosition == Position.LOW) {
                    hinge.goToSetPosition(VerticalHinge.Position.INTAKE, false, VS_HINGE_TO_INTAKE_TIME_LOW);
                } else {
                    hinge.goToSetPosition(VerticalHinge.Position.INTAKE, lastPosition == Position.INTAKE, VS_HINGE_TO_INTAKE_TIME);
                }
                openClaw();
                break;
            case LOW:
                slide.goToSetPosition(VerticalSlide.Position.LOW);
                lever.goToSetPosition(Lever.VerticalLeverPosition.LOW);
                // If position is currently intake, dont go immediately
                hinge.goToSetPosition(VerticalHinge.Position.LOW_UP, lastPosition != Position.INTAKE);
                closeClaw();
                break;
            case MEDIUM:
                slide.goToSetPosition(VerticalSlide.Position.MEDIUM);
                lever.goToSetPosition(Lever.VerticalLeverPosition.MEDIUM);
                // If position is currently intake, dont go immediately
                hinge.goToSetPosition(VerticalHinge.Position.MEDIUM_UP, lastPosition != Position.INTAKE);
                closeClaw();
                break;
            case HIGH:
                slide.goToSetPosition(VerticalSlide.Position.HIGH);
                lever.goToSetPosition(Lever.VerticalLeverPosition.HIGH);
                // If position is currently intake, dont go immediately
                hinge.goToSetPosition(VerticalHinge.Position.HIGH_UP, lastPosition != Position.INTAKE);
                closeClaw();
                break;
        }
        if (currentPosition == Position.INTAKE_LIMIT) {
            currentPosition = Position.INTAKE;
        }
    }

    public void setPower(double quote_un_quote_power) {
        slide.offsetPosition(quote_un_quote_power);
    }

    public void openClaw() {
        claw.open();
    }

    public void closeClaw() {
        claw.close();
    }

    public void toggleClaw() {
        claw.toggle();
    }

    public void toggleClawWide() {
        claw.toggleWide();
    }

    public void hingeUp() {
        switch (currentPosition) {
            case INTAKE:
                hinge.goToSetPosition(VerticalHinge.Position.INTAKE);
                break;
            case LOW:
                hinge.goToSetPosition(VerticalHinge.Position.LOW_UP);
                break;
            case MEDIUM:
                hinge.goToSetPosition(VerticalHinge.Position.MEDIUM_UP);
                break;
            case HIGH:
                hinge.goToSetPosition(VerticalHinge.Position.HIGH_UP);
                break;
        }
    }

    public void hingeDown() {
        switch (currentPosition) {
            case INTAKE:
                hinge.goToSetPosition(VerticalHinge.Position.INTAKE);
                break;
            case LOW:
                hinge.goToSetPosition(VerticalHinge.Position.LOW_DOWN);
                break;
            case MEDIUM:
                hinge.goToSetPosition(VerticalHinge.Position.MEDIUM_DOWN);
                break;
            case HIGH:
                hinge.goToSetPosition(VerticalHinge.Position.HIGH_DOWN);
                break;
        }
    }

    public Position getPosition() {
        return currentPosition;
    }

    public Position getLastPosition() {
        return lastPosition;
    }

    public boolean atPosition() {
        /* I don't think this is doing what I think it's supposed to be doing...
        if (lastPosition == getPosition()) {
            return true;
        }*/
        if (lastPosition == null || getPosition() == null) {
            return true;
        }
        return timer.time() > lastPosition.getWaitTime(getPosition());
        /*
        return slide.atSetPosition()
                && lever.atSetPosition(VS_LEVER_WAIT_TIME)
                && hinge.atSetPosition(VS_HINGE_WAIT_TIME)
                && claw.atSetPosition(VS_CLAW_WAIT_TIME);

         */
    }

    public void update() {
        hinge.update();
    }
}
