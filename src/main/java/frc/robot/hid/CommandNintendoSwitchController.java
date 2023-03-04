// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * For a controller that looks like a playstation controller but acts like an xbox.
 * A subclass of {@link XboxController} with {@link Trigger} factories for command-based.
 *
 * @see XboxController
 */
@SuppressWarnings({"MethodName", "unused"})
public class CommandNintendoSwitchController extends CommandGenericHID {
    private final NintendoSwitchController hid;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public CommandNintendoSwitchController(int port) {
        super(port);
        hid = new NintendoSwitchController(port);
    }

    /**
     * Get the underlying GenericHID object.
     *
     * @return the wrapped GenericHID object
     */
    @Override
    public NintendoSwitchController getHID() {
        return hid;
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @return an event instance representing the left bumper's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #leftBumper(EventLoop)
     */
    public Trigger leftBumper() {
        return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right bumper's digital signal attached to the given
     *     loop.
     */
    public Trigger leftBumper(EventLoop loop) {
        return hid.leftBumper(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @return an event instance representing the right bumper's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #rightBumper(EventLoop)
     */
    public Trigger rightBumper() {
        return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left bumper's digital signal attached to the given
     *     loop.
     */
    public Trigger rightBumper(EventLoop loop) {
        return hid.rightBumper(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @return an event instance representing the left stick button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #leftStick(EventLoop)
     */
    public Trigger leftStick() {
        return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left stick button's digital signal attached to the
     *     given loop.
     */
    public Trigger leftStick(EventLoop loop) {
        return hid.leftStick(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @return an event instance representing the right stick button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #rightStick(EventLoop)
     */
    public Trigger rightStick() {
        return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right stick button's digital signal attached to the
     *     given loop.
     */
    public Trigger rightStick(EventLoop loop) {
        return hid.rightStick(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @return an event instance representing the A button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #x(EventLoop)
     */
    public Trigger a() {
        return x(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the A button's digital signal attached to the given
     *     loop.
     */
    public Trigger a(EventLoop loop) {
        return hid.a(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @return an event instance representing the B button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #b(EventLoop)
     */
    public Trigger b() {
        return b(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the B button's digital signal attached to the given
     *     loop.
     */
    public Trigger b(EventLoop loop) {
        return hid.b(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @return an event instance representing the X button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #x(EventLoop)
     */
    public Trigger x() {
        return x(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the X button's digital signal attached to the given
     *     loop.
     */
    public Trigger x(EventLoop loop) {
        return hid.x(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @return an event instance representing the Y button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #y(EventLoop)
     */
    public Trigger y() {
        return y(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Y button's digital signal attached to the given
     *     loop.
     */
    public Trigger y(EventLoop loop) {
        return hid.y(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @return an event instance representing the start button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #home(EventLoop)
     */
    public Trigger home() {
        return home(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's digital signal attached to the given
     *     loop.
     */
    public Trigger home(EventLoop loop) {
        return hid.home(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @return an event instance representing the start button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #home(EventLoop)
     */
    public Trigger circle() {
        return circle(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's digital signal attached to the given
     *     loop.
     */
    public Trigger circle(EventLoop loop) {
        return hid.circle(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @return an event instance representing the back button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #plus(EventLoop)
     */
    public Trigger plus() {
        return plus(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the back button's digital signal attached to the given
     *     loop.
     */
    public Trigger plus(EventLoop loop) {
        return hid.plus(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @return an event instance representing the back button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #plus(EventLoop)
     */
    public Trigger minus() {
        return minus(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the back button's digital signal attached to the given
     *     loop.
     */
    public Trigger minus(EventLoop loop) {
        return hid.minus(loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
     *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *     button loop}.
     */
    public Trigger leftTrigger() {
        return leftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
     *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *     button loop}.
     */
    public Trigger leftTrigger(EventLoop loop) {
        return hid.leftTrigger(loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
     *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *     button loop}.
     */
    public Trigger rightTrigger() {
        return rightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
     *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *     button loop}.
     */
    public Trigger rightTrigger(EventLoop loop) {
        return hid.rightTrigger(loop).castTo(Trigger::new);
    }

    /**
     * Get the X axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftX() {
        return hid.getLeftX();
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightX() {
        return hid.getRightX();
    }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        return hid.getLeftY();
    }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightY() {
        return hid.getRightY();
    }
}
