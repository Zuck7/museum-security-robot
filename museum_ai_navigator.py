import os
import time
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from google import genai



# 1. AI CONFIGURATION
# Uses the GEMINI_API_KEY environment variable
# that we already configured in the terminal.

API_KEY = os.environ.get("GEMINI_API_KEY")

if not API_KEY:
    raise RuntimeError(
        "GEMINI_API_KEY is not set. "
        "Run: export GEMINI_API_KEY='YOUR_API_KEY'"
    )

client = genai.Client(api_key=API_KEY)


# 2. MUSEUM KNOWLEDGE BASE
# These are your colleague's original coordinates.
# We may need to adjust them to match YOUR saved map.

MUSEUM_LOCATIONS = {
    "lobby": {"x": 0.474, "y": 1.380},
    "dinosaur_exhibit": {"x": 2.800, "y": 2.619},
    "renaissance_statue": {"x": 0.513, "y": -1.653},
    "ai_innovation_wing": {"x": -1.573, "y": 0.491},
    "cafeteria": {"x": -0.477, "y": 2.502}
}



# 3. TRUE LLM INTENT PARSING
def llm_parse_intent(user_text):
    print(" Thinking...")

    prompt = f"""
You are the navigation brain for an autonomous security robot in a museum.

The museum has exactly these 5 locations:

1. "lobby"
2. "dinosaur_exhibit"
3. "renaissance_statue"
4. "ai_innovation_wing"
5. "cafeteria"

The user gave this command:

"{user_text}"

Based on the context of the user's command, reply with ONLY the
exact string of the location from the list above.

Do not include punctuation, explanation, or extra words.

Examples:

"I think someone is messing with the servers"
-> ai_innovation_wing

"Did I leave my backpack near the old bones?"
-> dinosaur_exhibit

"Someone is causing trouble where people eat"
-> cafeteria

"Check the statue"
-> renaissance_statue

"Someone suspicious just entered the building"
-> lobby

If the command does not make sense or does not match anything,
output "lobby".
"""

    try:
        response = client.models.generate_content(
            model="gemini-3.5-flash",
            contents=prompt
        )

        location = response.text.strip().lower()

        valid_locations = list(MUSEUM_LOCATIONS.keys())

        if location in valid_locations:
            return location

        print(
            f" LLM returned an invalid location: "
            f"'{location}'. Defaulting to lobby."
        )

        return "lobby"

    except Exception as e:
        print(f" API Error: {e}")
        return "lobby"


# 4. ROBOT EXECUTION
def main():

    rclpy.init()

    navigator = BasicNavigator()

    print(" Waiting for Nav2 to boot up...")

    navigator.waitUntilNav2Active()

    print("\n========================================================")
    print(" TRUE AI SECURITY BOT READY.")
    print("Try: 'I think someone is messing with the servers'")
    print("Try: 'Did I leave my backpack near the old bones?'")
    print("Type 'exit' to quit.")
    print("========================================================\n")

    while True:

        user_command = input("Commander > ").strip()

        if user_command.lower() in ["exit", "quit"]:
            break

        if not user_command:
            continue

        # Ask Gemini where the robot should go

        target_location = llm_parse_intent(user_command)

        print(
            f" AI concluded target is: "
            f"'{target_location}'"
        )

        coords = MUSEUM_LOCATIONS[target_location]

        # Create Nav2 goal

        goal_pose = PoseStamped()

        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = (
            navigator.get_clock().now().to_msg()
        )

        goal_pose.pose.position.x = coords["x"]
        goal_pose.pose.position.y = coords["y"]
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        print(
            f" Dispatching robot to "
            f"{target_location}"
        )

        print(
            f" Target coordinates: "
            f"X={coords['x']}, Y={coords['y']}\n"
        )

        # Send goal to Nav2

        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            time.sleep(0.1)

        # Check navigation result

        result = navigator.getResult()

        if result == TaskResult.SUCCEEDED:

            print(
                " Area secure. "
                "Ready for next command.\n"
            )

        elif result == TaskResult.CANCELED:

            print(
                " Navigation was canceled.\n"
            )

        else:

            print(
                " Navigation failed.\n"
            )

    navigator.destroyNode()
    rclpy.shutdown()


# 5. START PROGRAM

if __name__ == "__main__":
    main()
