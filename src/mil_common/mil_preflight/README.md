# Preflight

## Introduction
Preflight is an automated testing tool that should be run after turning on and connecting to the robot to run a prelaunch hardware checklist and automated software checklist.
Although it is a tui program, it supports standard mouse operation like click and scroll.

## Usage

1. Run the following command:
   ```bash
   source install/setup.bash
   mil_preflight ftx_ui
   ```
   This will start the preflight tui program.
2. In the main menu, select `Run Tests` and press Enter. This will open the **Run Tests** page.

3. The left side of the page lists four types of automated software tests:

   - **Actuators**: Prompts the user to enable physical moving actuators on the robot. Ensure the surrounding area is clear to avoid injury or damage. The user must visually confirm that the actuators behave as expected.
   
   - **Nodes**: A ROS Node is an active process performing a task. This test verifies that all required nodes are running and alive.
   
   - **Topics**: ROS Topics allow nodes to communicate via message publishing/subscribing. These tests confirm that expected data is being published, indicating that sensors are functioning correctly.
   
   - **Setup Tests**: Manual checks for features that cannot be automated (e.g., verifying that O-rings are greased).

   For each test type, the right side of the page lists multiple **specific test items**. You can select which test items to run by checking the box to the left of each test item name. If you want to run all test items under a specific test type, simply check the box next to the test type name.

4. Select test items you wish to run, then click the `Run` button at the bottom-right of the page.

5. Wait for the tests to complete. Respond to any prompts that appear during the process.

6. Test results will appear inline, to the right of each test name:
   - ✓ Success  
   - ✘ Failed

7. To view detailed test reports, go to the `View Reports` page by selecting `<` (top-left corner) to return to the main menu.

8. From the main menu, select `View Reports` and press Enter.

9. The most recent report will be displayed by default. Use `<` or `>` at the bottom-center of the page to navigate through previous reports.
   
