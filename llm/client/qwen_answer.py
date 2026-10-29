"""Cloud Qwen text priors via the OpenAI-compatible DashScope API."""
import json
import os

from llm.prompt.get_llm_answer import SYSTEM_PROMPT

# Include the prompt in the cache identity so changes invalidate old answers.
QWEN_PROMPT = SYSTEM_PROMPT + '''
Return ONLY a JSON object with keys:
"confidence": a number between 0.25 and 0.65,
"room": one short English room name or "everywhere".
Do not rewrite the target instruction. A room is only a prior, not an observed fact.
'''


def qwen_settings(config):
    return (os.getenv('QWEN_MODEL') or getattr(config, 'model', 'qwen-plus'),
            os.getenv('DASHSCOPE_BASE_URL') or getattr(config, 'base_url',
                'https://dashscope.aliyuncs.com/compatible-mode/v1'))


def qwen_respond(config, prompt):
    key = os.getenv('DASHSCOPE_API_KEY')
    if not key:
        raise ValueError('Set DASHSCOPE_API_KEY in the simulation container terminal.')
    from openai import OpenAI
    model, base_url = qwen_settings(config)
    client = OpenAI(api_key=key, base_url=base_url,
                    timeout=float(getattr(config, 'timeout', 30)), max_retries=0)
    try:
        completion = client.chat.completions.create(
            model=model,
            messages=[{'role': 'system', 'content': QWEN_PROMPT},
                      {'role': 'user', 'content': prompt}],
            response_format={'type': 'json_object'},
            extra_body={'enable_thinking': False}, stream=False)
        response = completion.choices[0].message.content
        data = json.loads(response)
        return [data['confidence'], data['room']], response
    finally:
        client.close()
