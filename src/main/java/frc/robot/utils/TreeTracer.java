// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class TreeTracer {
    private static final long MIN_PRINT_PERIOD = 1000000;

    private long lastEpochsPrintTime;

    private Node currentNode;

    public TreeTracer() {
        resetEpochs();
    }

    /** Clears all epochs. */
    public void resetEpochs() {
        long currentTime = RobotController.getFPGATime();
        currentNode = new Node("Base", currentTime, null);
    }

    public void addNode(String name) {
        Node newNode = new Node(name, RobotController.getFPGATime(), currentNode);
        currentNode.children.add(newNode);
        currentNode = newNode;
    }

    public void endCurrentNode() {
        currentNode.endTime = RobotController.getFPGATime();
        currentNode = currentNode.parent;
    }

    private void printNode(Node node, StringBuilder out, int indent) {
        out.append("\n");
        if (indent > 0) {
            out.append(" ".repeat(indent * 4 - 2)).append("↳ ");
        }
        out.append(node.name)
                .append(": ")
                .append(String.format("%.6f", (node.endTime - node.startTime) / 1000000.0))
                .append("s");
        for (Node child : node.children) {
            printNode(child, out, indent + 1);
        }
    }

    public void printEpochs() {
        printEpochs(out -> DriverStation.reportWarning(out, false));
    }

    public void printEpochs(Consumer<String> output) {
        long now = RobotController.getFPGATime();
        if (now - lastEpochsPrintTime > MIN_PRINT_PERIOD) {
            lastEpochsPrintTime = now;
            StringBuilder sb = new StringBuilder();
            for (Node child : currentNode.children) {
                printNode(child, sb, 0);
            }
            if (sb.length() > 0) {
                output.accept(sb.toString());
            }
        }
    }

    static class Node {
        private final String name;
        private final long startTime;
        private long endTime;
        private final Node parent;
        private final List<Node> children = new ArrayList<>();

        Node(String name, long startTime, Node parent) {
            this.name = name;
            this.startTime = startTime;
            this.parent = parent;
        }
    }
}
