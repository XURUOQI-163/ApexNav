"""Target room prior and target confidence threshold, without auxiliary classes."""
SYSTEM_PROMPT = """You provide navigation context for a target object description.
Return the most likely room only when it is substantially more likely than other
rooms; otherwise use "everywhere". Use a short English room name. This is a
common-sense prior, not a claim about the observed scene.
Also choose a target confidence threshold between 0.25 and 0.65: small objects
such as an apple use a lower threshold (0.25), large objects such as a bed use
higher thresholds (0.65). Preserve the meaning of descriptive target attributes.
Output exactly: Answer: [confidence, "room"]
"""
USER1 = "cake"
ASSISTANT1 = 'Answer: [0.30, "everywhere"]'
USER2 = "bed"
ASSISTANT2 = 'Answer: [0.65, "bedroom"]'
