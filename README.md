# Turtle_Bot3_trackFollow
## Overview

This project implements a modular autonomous autorace system for TurtleBot3, where track following, obstacle avoidance, and road sign navigation are split into independent executable ROS2 nodes.

The robot follows a colored track using camera-based perception, avoids obstacles using LiDAR, and reacts to contextual road signs such as tunnel entry using service-based behavior control. The system is containerized using multiple Docker Compose configurations for flexible deployment.

## System Architecture & Flow

The project is divided into independent tasks, each executed as a separate ROS2 node:

### High-Level Flow

1. Track Following Node
    Follows dual colored track lines using camera input.
   
3. Obstacle Avoidance Node
    Triggered when obstacles are detected via LiDAR.
    Uses action-based navigation to bypass obstacles.
   
5. Track Recovery
    After avoiding an obstacle, the robot searches for track lines.
    Once detected, it realigns itself and resumes track following.
   
7. Sign-Based Navigation
    Tunnel sign detection is handled separately using a service node.
    On detection, a tunnel-specific navigation behavior is triggered.

## Track Following (Vision-Based)

 Implemented using OpenCV and the onboard camera.
 Detects two colored striped lines forming the track.
 Robot behavior:

   Follows both lines when visible.
   If one line is lost, adjusts heading using the remaining line.
   Recovers once both lines are detected again.

## Obstacle Avoidance (LiDAR-Based)

 Implemented as a separate executable node.
 Uses LiDAR data to detect obstacles.
 Executes an action-based navigation routine to bypass obstacles.
 Continues until track lines are re-detected.
 Hands control back to the track-following node after realignment.

## Tunnel Sign Recognition & Navigation

 Implemented as a service-based node.
 Detects tunnel entry signs using:
   OpenCV feature matching

 On successful detection:
   Triggers tunnel-specific navigation behavior
   Modifies robot motion and navigation logic

## Docker-Based Execution

The system is deployed using two separate Docker Compose files, depending on the task.

### Track Following + Obstacle Avoidance

Docker Compose file:
follow_track_with_obstacle_avoidance.yml

Executables launched:
 `follow_both_line`
 `obstacle_avoid_navigation`

    docker-compose -f follow_track_with_obstacle_avoidance.yml up




### Tunnel Sign Recognition & Navigation

Docker Compose file:
tunnel_navigation.yml


Executable launched:
 `tunnel_navigation_srv`

    docker-compose -f tunnel_navigation.yml up
