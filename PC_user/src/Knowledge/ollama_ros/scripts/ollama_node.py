#!/usr/bin/env python3
import rospy
from ollama_ros.srv import CommandPlan, CommandPlanResponse
import ollama
import json
import re

def extract_first_json_block(text):
    # Busca el primer bloque JSON válido usando regex
    json_blocks = re.findall(r'\{[\s\S]*?\}', text)
    if not json_blocks:
        raise ValueError("No JSON block found in response")
    return json_blocks[0]

def parse_command_with_phi3(command_text):
    prompt = f"""
You are a command parser for a home robot. Given a user command in English or Spanish, 
output ONLY a JSON object with these exact fields:

{{
  "action": "main action (string, e.g., 'count', 'navigate', 'tell', 'follow', 'take', 'give', 'greet', 'answer', 'describe', 'offer', 'guide', 'meet', 'remember', 'place')",
  "target": "main object or person (string, e.g., 'people', 'bottle', 'Morgan', 'everyone', 'the person wearing red')",
  "details": "specific details or conditions (string, e.g., 'wearing red t-shirts', 'on the kitchen table'; if none, use '')",
  "location": "origin or where to perform the action (string, e.g., 'kitchen', 'living room'; if none, use '')",
  "destination": "if moving, where to go or deliver (string, e.g., 'bathroom', 'bring to me'; if none, use '')",
  "secondary_action": "if there's a second action, describe it here (string, e.g., 'then tell me how many people are in the room'; if none, use '')"
}}

⚠ IMPORTANT:
- Always fill **all** fields, even if empty (`""`).
- If the command contains 'how many', 'how much', or requests a count, set `action` to 'count'.
- If the command involves following someone, use 'follow' and capture both `location` (origin) and `destination`.
- If the command involves fetching or delivering something, use 'take', 'give', or 'deliver' as appropriate.
- If the command has sequential tasks (e.g., 'go to the kitchen then bring me water'), put the first action in `action` and the second in `secondary_action`.
- Support both English and Spanish commands, mapping verbs and objects properly.
- Only return a single valid JSON block, no explanations, no markdown, no comments.

Command: {command_text}
"""




    try:
        response = ollama.chat(model='phi3:mini', messages=[
            {'role': 'user', 'content': prompt}
        ])
        raw_content = response['message']['content'].strip()
        # Extrae el primer bloque JSON
        json_text = extract_first_json_block(raw_content)
        parsed = json.loads(json_text)
        return json.dumps(parsed)
    except Exception as e:
        rospy.logerr(f"Parsing failed: {e}")
        return json.dumps({"error": "failed to parse"})

def handle_parse_command(req):
    rospy.loginfo(f"Received command: {req.command}")
    plan = parse_command_with_phi3(req.command)
    return CommandPlanResponse(plan)

def command_parser_server():
    rospy.init_node('command_parser_server')
    service = rospy.Service('parse_command', CommandPlan, handle_parse_command)
    rospy.loginfo("Command parser service ready.")
    rospy.spin()

if __name__ == "__main__":
    command_parser_server()
