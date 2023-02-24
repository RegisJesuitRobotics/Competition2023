// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class NintendoSwitchController extends GenericHID {
    public enum Button {
        LeftBumper(5),
        RightBumper(6),
        LeftTrigger(7),
        RightTrigger(8),
        Minus(9),
        Plus(10),
        LeftStick(11),
        RightStick(12),
        Y(1),
        B(2),
        A(3),
        X(4),
        Circle(13),
        Home(14);

        public final int value;

        Button(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods. This is done by
         * stripping the leading `k`, and if not a Bumper button append `Button`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (name.endsWith("Bumper")) {
                return name;
            }
            return name + "Button";
        }
    }

    /** Represents an axis on an XboxController. */
    public enum Axis {
        LeftX(0),
        RightX(2),
        LeftY(1),
        RightY(3);

        public final int value;

        Axis(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the axis, matching the relevant methods. This is done by
         * stripping the leading `k`, and if a trigger axis append `Axis`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the axis.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (name.endsWith("Trigger")) {
                return name + "Axis";
            }
            return name;
        }
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public NintendoSwitchController(final int port) {
        super(port);
    }

    /**
     * Get the X axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftX() {
        return getRawAxis(Axis.LeftX.value);
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightX() {
        return getRawAxis(Axis.RightX.value);
    }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        return getRawAxis(Axis.LeftY.value);
    }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightY() {
        return getRawAxis(Axis.RightY.value);
    }

    /**
     * Read the value of the left bumper (LB) button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLeftBumper() {
        return getRawButton(Button.LeftBumper.value);
    }

    /**
     * Read the value of the right bumper (RB) button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getRightBumper() {
        return getRawButton(Button.RightBumper.value);
    }

    /**
     * Whether the left bumper (LB) was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLeftBumperPressed() {
        return getRawButtonPressed(Button.LeftBumper.value);
    }

    /**
     * Whether the right bumper (RB) was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getRightBumperPressed() {
        return getRawButtonPressed(Button.RightBumper.value);
    }

    /**
     * Whether the left bumper (LB) was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getLeftBumperReleased() {
        return getRawButtonReleased(Button.LeftBumper.value);
    }

    /**
     * Whether the right bumper (RB) was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getRightBumperReleased() {
        return getRawButtonReleased(Button.RightBumper.value);
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right bumper's digital signal attached to the given
     *     loop.
     */
    public BooleanEvent leftBumper(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftBumper);
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left bumper's digital signal attached to the given
     *     loop.
     */
    public BooleanEvent rightBumper(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightBumper);
    }

    /**
     * Read the value of the left stick button (LSB) on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLeftStickButton() {
        return getRawButton(Button.LeftStick.value);
    }

    /**
     * Read the value of the right stick button (RSB) on the controller.
     *
     * @return The state of the button.
     */
    public boolean getRightStickButton() {
        return getRawButton(Button.RightStick.value);
    }

    /**
     * Whether the left stick button (LSB) was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLeftStickButtonPressed() {
        return getRawButtonPressed(Button.LeftStick.value);
    }

    /**
     * Whether the right stick button (RSB) was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getRightStickButtonPressed() {
        return getRawButtonPressed(Button.RightStick.value);
    }

    /**
     * Whether the left stick button (LSB) was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getLeftStickButtonReleased() {
        return getRawButtonReleased(Button.LeftStick.value);
    }

    /**
     * Whether the right stick (RSB) button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getRightStickButtonReleased() {
        return getRawButtonReleased(Button.RightStick.value);
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left stick button's digital signal attached to the
     *     given loop.
     */
    public BooleanEvent leftStick(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftStickButton);
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right stick button's digital signal attached to the
     *     given loop.
     */
    public BooleanEvent rightStick(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightStickButton);
    }

    /**
     * Read the value of the A button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getAButton() {
        return getRawButton(Button.A.value);
    }

    /**
     * Whether the A button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getAButtonPressed() {
        return getRawButtonPressed(Button.A.value);
    }

    /**
     * Whether the A button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getAButtonReleased() {
        return getRawButtonReleased(Button.A.value);
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the A button's digital signal attached to the given
     *     loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent a(EventLoop loop) {
        return new BooleanEvent(loop, this::getAButton);
    }

    /**
     * Read the value of the B button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBButton() {
        return getRawButton(Button.B.value);
    }

    /**
     * Whether the B button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBButtonPressed() {
        return getRawButtonPressed(Button.B.value);
    }

    /**
     * Whether the B button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBButtonReleased() {
        return getRawButtonReleased(Button.B.value);
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the B button's digital signal attached to the given
     *     loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent b(EventLoop loop) {
        return new BooleanEvent(loop, this::getBButton);
    }

    /**
     * Read the value of the X button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getXButton() {
        return getRawButton(Button.X.value);
    }

    /**
     * Whether the X button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getXButtonPressed() {
        return getRawButtonPressed(Button.X.value);
    }

    /**
     * Whether the X button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getXButtonReleased() {
        return getRawButtonReleased(Button.X.value);
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the X button's digital signal attached to the given
     *     loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent x(EventLoop loop) {
        return new BooleanEvent(loop, this::getXButton);
    }

    /**
     * Read the value of the Y button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getYButton() {
        return getRawButton(Button.Y.value);
    }

    /**
     * Whether the Y button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getYButtonPressed() {
        return getRawButtonPressed(Button.Y.value);
    }

    /**
     * Whether the Y button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getYButtonReleased() {
        return getRawButtonReleased(Button.Y.value);
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Y button's digital signal attached to the given
     *     loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent y(EventLoop loop) {
        return new BooleanEvent(loop, this::getYButton);
    }

    /**
     * Read the value of the minus button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getMinusButton() {
        return getRawButton(Button.Minus.value);
    }

    /**
     * Whether the back button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getMinusButtonPressed() {
        return getRawButtonPressed(Button.Minus.value);
    }

    /**
     * Whether the back button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getMinusButtonReleased() {
        return getRawButtonReleased(Button.Minus.value);
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the back button's digital signal attached to the given
     *     loop.
     */
    public BooleanEvent minus(EventLoop loop) {
        return new BooleanEvent(loop, this::getMinusButton);
    }

    /**
     * Read the value of the Plus button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getPlusButton() {
        return getRawButton(Button.Plus.value);
    }

    /**
     * Whether the Plus button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getPlusButtonPressed() {
        return getRawButtonPressed(Button.Plus.value);
    }

    /**
     * Whether the Plus button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getPlusButtonReleased() {
        return getRawButtonReleased(Button.Plus.value);
    }

    /**
     * Constructs an event instance around the Plus button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Plus button's digital signal attached to the given
     *     loop.
     */
    public BooleanEvent plus(EventLoop loop) {
        return new BooleanEvent(loop, this::getPlusButton);
    }

    /**
     * Read the value of the Plus button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getHomeButton() {
        return getRawButton(Button.Home.value);
    }

    /**
     * Whether the Plus button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getHomeButtonPressed() {
        return getRawButtonPressed(Button.Home.value);
    }

    /**
     * Whether the Plus button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getHomeButtonReleased() {
        return getRawButtonReleased(Button.Home.value);
    }

    /**
     * Constructs an event instance around the Plus button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Plus button's digital signal attached to the given
     *     loop.
     */
    public BooleanEvent home(EventLoop loop) {
        return new BooleanEvent(loop, this::getHomeButton);
    }

    /**
     * Read the value of the Plus button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getCircleButton() {
        return getRawButton(Button.Circle.value);
    }

    /**
     * Whether the Plus button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getCircleButtonPressed() {
        return getRawButtonPressed(Button.Circle.value);
    }

    /**
     * Whether the Plus button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getCircleButtonReleased() {
        return getRawButtonReleased(Button.Circle.value);
    }

    /**
     * Constructs an event instance around the Plus button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Plus button's digital signal attached to the given
     *     loop.
     */
    public BooleanEvent circle(EventLoop loop) {
        return new BooleanEvent(loop, this::getPlusButton);
    }

    /**
     * Read the value of the Plus button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLeftTriggerButton() {
        return getRawButton(Button.LeftTrigger.value);
    }

    /**
     * Whether the Plus button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLeftTriggerButtonPressed() {
        return getRawButtonPressed(Button.LeftTrigger.value);
    }

    /**
     * Whether the Plus button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getLeftTriggerButtonReleased() {
        return getRawButtonReleased(Button.LeftTrigger.value);
    }

    /**
     * Constructs an event instance around the Plus button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Plus button's digital signal attached to the given
     *     loop.
     */
    public BooleanEvent leftTrigger(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftTriggerButton);
    }

    /**
     * Read the value of the Plus button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getRightTriggerButton() {
        return getRawButton(Button.RightTrigger.value);
    }

    /**
     * Whether the Plus button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getRightTriggerButtonPressed() {
        return getRawButtonPressed(Button.RightTrigger.value);
    }

    /**
     * Whether the Plus button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getRightTriggerButtonReleased() {
        return getRawButtonReleased(Button.RightTrigger.value);
    }

    /**
     * Constructs an event instance around the Plus button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Plus button's digital signal attached to the given
     *     loop.
     */
    public BooleanEvent rightTrigger(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightTriggerButton);
    }
}
