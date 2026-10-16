# AI-assisted Recycling Sorter

ROS 2 Humble / Gazebo Classic demo for a small mobile robot that identifies color-coded recyclable packages with a local nearest-centroid image classifier, carries each package in a simulated virtual gripper, and places it in a category-specific bin.

The classifier is trained from the included sample CSV; it uses no API key, network service, or downloaded model. Package color is a visible marker for the category in this prototype, not a claim that color alone can identify real-world waste.

## Project contents

- `ros_ws/src/pa1_sortbot`: ROS 2 package, Gazebo world/model, launch file, and tests.
- `docker-compose.yml`: course Humble image with the workspace mounted into the container.
- `report.md`: Japanese report draft with no personal information.
- `assignment_demo.mp4`: clean 2-minute-40-second recording of the complete Gazebo mission (H.264, 1920×990).
- `scripts/generate_report.ps1`: Windows PowerShell script to create a PDF from the report.

Run the PDF generator from PowerShell before submission:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\generate_report.ps1
```

It uses the course container's ReportLab installation to create a searchable Japanese PDF with the required filename pattern. The generated filename is `説明資料_PAI最終課題_MochiMomo.pdf`.

## Run with the course Docker image

Install and start Docker Desktop, then from this directory run:

```powershell
docker compose up -d
docker compose exec sim bash
```

In the container terminal:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/ros_ws
colcon build --symlink-install --packages-select pa1_sortbot
source install/setup.bash
colcon test --packages-select pa1_sortbot
ros2 launch pa1_sortbot sort_demo.launch.py
```

Open the course desktop at `http://localhost:6080/` to watch Gazebo. The robot starts from the left, aligns its camera with each of three packages, and moves each recognized package to its matching bin. It stops just before each bin and uses Gazebo's `set_entity_state` service to place the virtual-gripped item inside. Packages are collision-free visual markers so they cannot obstruct the approach. This makes the task reproducible without a physical manipulator, but is not force-based grasping.

On CPU-constrained machines, run the same mission without the Gazebo client using `ros2 launch pa1_sortbot sort_demo.launch.py gui:=false`.

If the course image is not already available, Docker Compose will pull `airobotbook/ros2-desktop-ai-robot-book-humble:latest`. The first pull can take several minutes. Stop the environment with `docker compose down`.

## Record the assignment video

The included `assignment_demo.mp4` shows the complete three-item Gazebo run in a 2-minute-40-second clip. It is captured directly from the Gazebo window without desktop overlays. The video has not been uploaded to YouTube; upload it as unlisted if a video link is required for submission. Avoid showing names, account details, email addresses, or other personal information.

## Local checks

The classifier and movement helpers use only Python's standard library and can be tested in the course container:

```bash
python3 -m unittest discover -s /workspace/ros_ws/src/pa1_sortbot/test -v
```

The ROS integration test requires the course image with ROS 2 Humble and Gazebo Classic. See `report.md` for task design, AI method, limitations, and next steps.
