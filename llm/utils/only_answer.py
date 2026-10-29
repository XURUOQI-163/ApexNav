"""Parse the room/threshold response used by Ollama and DeepSeek."""
import ast
import re


def only_answer(response):
    match = re.search(r'Answer:\s*(\[[^\]]+\])', response)
    if not match:
        raise ValueError('Expected Answer: [confidence, "room"]')
    return ast.literal_eval(match.group(1))
