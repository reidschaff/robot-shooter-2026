/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.grabber;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;

public class GrabberCommands {
    public static Pair<Command, Command> createGrabberCommands(Grabber grabber) {
        Command onTrue = grabber.runOnce(grabber::transitionToCollecting);
        Command onFalse = grabber.runOnce(grabber::transitionToDisabled);

        return Pair.of(onTrue, onFalse);
    }

    public static Pair<Command, Command> createGrabberScoringCommands(Grabber grabber) {
        Command onTrue = grabber.runOnce(grabber::transitionToScoring);
        Command onFalse = grabber.runOnce(grabber::transitionToDisabled).onlyIf(() -> !grabber.collectionTimer.isRunning());

        return Pair.of(onTrue, onFalse);
    }
}
