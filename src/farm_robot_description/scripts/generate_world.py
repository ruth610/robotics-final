#!/usr/bin/env python3
import os
import random

def generate_sdf():
    # Header
    sdf = """<?xml version='1.0'?>
<sdf version='1.6'>
  <world name='farm_world'>
    <gravity>0 0 -9.8</gravity>
    <physics type='ode'>
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Brown Soil Ground -->
    <model name="soil_ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Mud</name>
            </script>
            <!-- Fallback color if texture fails -->
            <ambient>0.45 0.35 0.23 1</ambient>
            <diffuse>0.45 0.35 0.23 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

    # Generate Plants
    rows_y = [-1.5, -0.5, 0.5, 1.5]
    start_x = -5.0
    end_x = 5.0
    step_x = 0.5

    plant_count = 0
    current_x = start_x

    while current_x <= end_x:
        for y in rows_y:
            # Randomly decide plant health
            is_healthy = random.random() > 0.15  # 15% chance of stress

            # Random rotation for realism
            yaw = random.uniform(0, 3.14)

            if is_healthy:
                stem_color = "0.1 0.5 0.1 1" # Dark Green
                leaf_color = "0.2 0.7 0.2 1" # Bright Green
            else:
                stem_color = "0.6 0.5 0.3 1" # Brownish
                leaf_color = "0.8 0.7 0.1 1" # Yellow/Withered

            sdf += f"""
    <model name='plant_{plant_count}'>
      <pose>{current_x} {y} 0 0 0 {yaw}</pose>
      <static>true</static>
      <link name='link'>
        <!-- Stem -->
        <visual name='stem'>
          <pose>0 0 0.1 0 0 0</pose>
          <geometry>
            <cylinder>
                <radius>0.015</radius>
                <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{stem_color}</ambient>
            <diffuse>{stem_color}</diffuse>
          </material>
        </visual>

        <!-- Leaves (Crossed Planes) -->
        <visual name='leaf_1'>
          <pose>0 0 0.15 0 0.3 0</pose>
          <geometry>
            <box><size>0.15 0.04 0.005</size></box>
          </geometry>
          <material>
             <ambient>{leaf_color}</ambient>
             <diffuse>{leaf_color}</diffuse>
          </material>
        </visual>
         <visual name='leaf_2'>
          <pose>0 0 0.15 0 -0.3 0</pose>
          <geometry>
            <box><size>0.15 0.04 0.005</size></box>
          </geometry>
          <material>
             <ambient>{leaf_color}</ambient>
             <diffuse>{leaf_color}</diffuse>
          </material>
        </visual>
         <visual name='leaf_3'>
          <pose>0 0 0.12 0.3 0 1.57</pose>
          <geometry>
            <box><size>0.15 0.04 0.005</size></box>
          </geometry>
          <material>
             <ambient>{leaf_color}</ambient>
             <diffuse>{leaf_color}</diffuse>
          </material>
        </visual>

        <collision name='collision'>
          <pose>0 0 0.1 0 0 0</pose>
          <geometry>
             <cylinder>
                <radius>0.05</radius>
                <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>
"""
            plant_count += 1
        current_x += step_x

    # Close SDF
    sdf += "\n  </world>\n</sdf>"

    return sdf

if __name__ == "__main__":
    content = generate_sdf()
    output_path = "src/farm_robot_description/worlds/farm.world"
    # Ensure directory exists (it should)
    abs_path = os.path.abspath(output_path)
    with open(abs_path, 'w') as f:
        f.write(content)
    print(f"Generated world file at {abs_path}")
